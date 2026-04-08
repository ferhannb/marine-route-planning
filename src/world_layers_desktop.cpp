#include "world_layers_support.h"

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QFutureWatcher>
#include <QGraphicsScene>
#include <QGraphicsPathItem>
#include <QGraphicsSvgItem>
#include <QGraphicsView>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QList>
#include <QLineEdit>
#include <QPainterPath>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QTimer>
#include <QRubberBand>
#include <QScrollArea>
#include <QShortcut>
#include <QSvgRenderer>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QtConcurrent/QtConcurrentRun>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace {

std::uint64_t desktopEdgeKey(int a, int b) {
    const std::uint32_t u = static_cast<std::uint32_t>(std::min(a, b));
    const std::uint32_t v = static_cast<std::uint32_t>(std::max(a, b));
    return (static_cast<std::uint64_t>(u) << 32U) | v;
}

struct RenderTaskResult {
    bool success = false;
    std::string error_message;
    std::string warning_message;
    std::string resolved_svg_path;
    worldlayers::Config config;
    worldlayers::Summary summary;
    bool has_geo_transform = false;
    double min_lon = 0.0;
    double max_lon = 0.0;
    double min_lat = 0.0;
    double max_lat = 0.0;
    double plot_x = 0.0;
    double plot_y = 0.0;
    double plot_w = 0.0;
    double plot_h = 0.0;
    double canvas_w = 1400.0;
    double canvas_h = 900.0;
};

struct TriangulationLayerTaskResult {
    bool success = false;
    std::string error_message;
    QPainterPath path;
    std::uint64_t triangle_count = 0;
    std::uint64_t edge_count = 0;
};

struct RoutePlanningTaskResult {
    bool success = false;
    bool found = false;
    bool used_tss_centerline = false;
    std::string error_message;
    std::string warning_message;
    planner::LatLon start_geo{0.0, 0.0};
    planner::LatLon goal_geo{0.0, 0.0};
    std::vector<planner::LatLon> route_polyline;
    std::vector<int> triangle_sequence;
    double total_km = 0.0;
    int mesh_start_vertex = -1;
    int mesh_goal_vertex = -1;
};

QPointF projectGeoToSvgPoint(const planner::LatLon& geo, const RenderTaskResult& transform) {
    const double lon_t = (geo.lon_deg - transform.min_lon) / (transform.max_lon - transform.min_lon);
    const double lat_t = (transform.max_lat - geo.lat_deg) / (transform.max_lat - transform.min_lat);
    const double x = transform.plot_x + lon_t * transform.plot_w;
    const double y = transform.plot_y + lat_t * transform.plot_h;
    return {x, y};
}

QPainterPath makePinPath() {
    QPainterPath path;
    path.setFillRule(Qt::OddEvenFill);
    path.moveTo(0.0, 0.0);
    path.cubicTo(-7.0, -8.0, -7.0, -18.0, 0.0, -22.0);
    path.cubicTo(7.0, -18.0, 7.0, -8.0, 0.0, 0.0);
    path.closeSubpath();
    path.addEllipse(QPointF(0.0, -12.0), 3.2, 3.2);
    return path;
}

RenderTaskResult makeGeoTransformResult(const worldlayers::LoadedData& data) {
    RenderTaskResult result;
    double min_lon = std::numeric_limits<double>::infinity();
    double max_lon = -std::numeric_limits<double>::infinity();
    double min_lat = std::numeric_limits<double>::infinity();
    double max_lat = -std::numeric_limits<double>::infinity();

    auto extendBounds = [&](double lat, double lon) {
        min_lon = std::min(min_lon, lon);
        max_lon = std::max(max_lon, lon);
        min_lat = std::min(min_lat, lat);
        max_lat = std::max(max_lat, lat);
    };

    for (const auto& poly : data.land) {
        extendBounds(poly.min_lat, poly.min_lon);
        extendBounds(poly.max_lat, poly.max_lon);
    }
    for (const auto& f : data.tss) {
        for (const auto& p : f.points) {
            extendBounds(p.lat_deg, p.lon_deg);
        }
    }
    for (const auto& f : data.bathymetry) {
        for (const auto& p : f.points) {
            extendBounds(p.lat_deg, p.lon_deg);
        }
    }
    for (const auto& zone : data.buffer_zones) {
        extendBounds(zone.polygon.min_lat, zone.polygon.min_lon);
        extendBounds(zone.polygon.max_lat, zone.polygon.max_lon);
    }

    if (!std::isfinite(min_lon) || !std::isfinite(max_lon) || !std::isfinite(min_lat) || !std::isfinite(max_lat)) {
        return result;
    }

    constexpr double width = 1400.0;
    constexpr double height = 900.0;
    constexpr double margin = 70.0;

    const double lon_span = std::max(max_lon - min_lon, 1.0);
    const double lat_span = std::max(max_lat - min_lat, 1.0);
    min_lon -= lon_span * 0.06;
    max_lon += lon_span * 0.06;
    min_lat -= lat_span * 0.06;
    max_lat += lat_span * 0.06;

    const double drawable_w = width - 2.0 * margin;
    const double drawable_h = height - 2.0 * margin;
    const double sx = drawable_w / (max_lon - min_lon);
    const double sy = drawable_h / (max_lat - min_lat);
    const double scale = std::min(sx, sy);
    const double plot_w = (max_lon - min_lon) * scale;
    const double plot_h = (max_lat - min_lat) * scale;
    const double plot_x = margin + 0.5 * (drawable_w - plot_w);
    const double plot_y = margin + 0.5 * (drawable_h - plot_h);

    result.has_geo_transform = true;
    result.min_lon = min_lon;
    result.max_lon = max_lon;
    result.min_lat = min_lat;
    result.max_lat = max_lat;
    result.plot_x = plot_x;
    result.plot_y = plot_y;
    result.plot_w = plot_w;
    result.plot_h = plot_h;
    result.canvas_w = width;
    result.canvas_h = height;
    return result;
}

bool loadLandForPlannerCache(const planner::PlannerCacheData& cache,
                             std::vector<planner::LandPolygon>& out_land,
                             std::string& error_message) {
    if (!cache.land_polygons.empty()) {
        out_land = cache.land_polygons;
        return true;
    }

    const std::string resolved_land_dataset = worldlayers::resolveInputPath(cache.dataset_path);
    if (resolved_land_dataset.empty()) {
        error_message = "No land dataset information found in the cache.";
        return false;
    }

    if (!planner::loadLandPolygonsGeoJson(resolved_land_dataset, out_land) &&
        !planner::loadLandPolygons(resolved_land_dataset, out_land)) {
        error_message = "Failed to load land dataset.";
        return false;
    }
    return true;
}

bool loadBathymetryForPlannerCache(const planner::PlannerCacheData& cache,
                                   std::vector<planner::BathymetryFeature>& out_bathymetry,
                                   std::string& error_message) {
    out_bathymetry.clear();
    const std::string resolved_bathymetry_dataset = worldlayers::resolveInputPath(cache.bathymetry_path);
    if (resolved_bathymetry_dataset.empty()) {
        return true;
    }

    if (!planner::loadBathymetryFeaturesGeoJson(resolved_bathymetry_dataset, out_bathymetry) &&
        !planner::loadBathymetryFeatures(resolved_bathymetry_dataset, out_bathymetry)) {
        error_message = "Failed to load bathymetry dataset.";
        return false;
    }
    return true;
}

planner::TrafficRoutingConfig defaultDesktopTrafficRoutingConfig() {
    planner::TrafficRoutingConfig cfg;
    cfg.enable_tss_routing = true;
    cfg.hard_tss_corridor = true;
    cfg.lane_preference_multiplier = 0.82;
    cfg.off_lane_multiplier = 1.30;
    cfg.separation_zone_penalty_km = 8.0;
    cfg.boundary_crossing_penalty_km = 3.0;
    cfg.lane_corridor_half_width_km = 1.0;
    cfg.lane_endpoint_radius_km = 3.0;
    return cfg;
}

class ZoomGraphicsView : public QGraphicsView {
public:
    explicit ZoomGraphicsView(QWidget* parent = nullptr) : QGraphicsView(parent) {
        setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
        setDragMode(QGraphicsView::ScrollHandDrag);
        setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
        setResizeAnchor(QGraphicsView::AnchorViewCenter);
    }

protected:
    void mousePressEvent(QMouseEvent* event) override {
        selection_origin_ = event->pos();
        if ((event->modifiers() & Qt::ControlModifier) != 0 && event->button() == Qt::LeftButton) {
            if (!selection_band_) {
                selection_band_ = std::make_unique<QRubberBand>(QRubberBand::Rectangle, viewport());
            }
            selection_band_->setGeometry(QRect(selection_origin_, QSize()));
            selection_band_->show();
            ctrl_selecting_ = true;
            event->accept();
            return;
        }
        if (point_pick_enabled_ && event->button() == Qt::LeftButton) {
            event->accept();
            return;
        }
        QGraphicsView::mousePressEvent(event);
    }

    void mouseMoveEvent(QMouseEvent* event) override {
        if (ctrl_selecting_ && selection_band_) {
            selection_band_->setGeometry(QRect(selection_origin_, event->pos()).normalized());
            event->accept();
            return;
        }
        QGraphicsView::mouseMoveEvent(event);
    }

    void wheelEvent(QWheelEvent* event) override {
        constexpr double kZoomIn = 1.15;
        constexpr double kZoomOut = 1.0 / kZoomIn;
        if (event->angleDelta().y() > 0) {
            scale(kZoomIn, kZoomIn);
        } else {
            scale(kZoomOut, kZoomOut);
        }
        event->accept();
    }

public:
    void setCtrlDragZoomHandler(std::function<void(const QRectF&)> handler) {
        ctrl_drag_zoom_handler_ = std::move(handler);
    }

    void setRightClickHandler(std::function<void()> handler) {
        right_click_handler_ = std::move(handler);
    }

    void setPointPickHandler(std::function<void(const QPointF&)> handler) {
        point_pick_handler_ = std::move(handler);
    }

    void setPointPickEnabled(bool enabled) { point_pick_enabled_ = enabled; }

private:
    void mouseReleaseEvent(QMouseEvent* event) override {
        if (ctrl_selecting_ && event->button() == Qt::LeftButton) {
            ctrl_selecting_ = false;
            if (selection_band_) {
                const QRect view_rect = selection_band_->geometry().normalized();
                selection_band_->hide();
                if (view_rect.width() >= 8 && view_rect.height() >= 8 && ctrl_drag_zoom_handler_) {
                    const QRectF scene_rect = mapToScene(view_rect).boundingRect();
                    ctrl_drag_zoom_handler_(scene_rect);
                }
            }
            event->accept();
            return;
        }
        if (point_pick_enabled_ && event->button() == Qt::LeftButton && point_pick_handler_) {
            const QPoint delta = event->pos() - selection_origin_;
            if (std::abs(delta.x()) <= 4 && std::abs(delta.y()) <= 4) {
                point_pick_handler_(mapToScene(event->pos()));
                event->accept();
                return;
            }
        }
        if (event->button() == Qt::RightButton && right_click_handler_) {
            right_click_handler_();
            event->accept();
            return;
        }
        QGraphicsView::mouseReleaseEvent(event);
    }

    std::function<void(const QRectF&)> ctrl_drag_zoom_handler_;
    std::function<void()> right_click_handler_;
    std::function<void(const QPointF&)> point_pick_handler_;
    std::unique_ptr<QRubberBand> selection_band_;
    QPoint selection_origin_;
    bool ctrl_selecting_ = false;
    bool point_pick_enabled_ = false;
};

class ViewerWindow : public QWidget {
public:
    explicit ViewerWindow(QString startup_screenshot_path = QString())
        : startup_screenshot_path_(std::move(startup_screenshot_path)) {
        setWindowFlags(windowFlags() | Qt::Window | Qt::WindowMinMaxButtonsHint | Qt::WindowFullscreenButtonHint);
        setWindowTitle("VGPathPlanning World Layers Desktop");
        resize(1600, 920);
        buildUi();
        loadDefaults();
        renderNow();
    }

private:
    enum class PickMode {
        None,
        Start,
        Goal,
    };

    static QString defaultBathymetryDatasetPath() {
        const std::filesystem::path data_dir(worldlayers::resolveInputPath("dataset"));
        if (std::filesystem::exists(data_dir) && std::filesystem::is_directory(data_dir)) {
            for (const auto& entry : std::filesystem::directory_iterator(data_dir)) {
                if (!entry.is_regular_file()) {
                    continue;
                }
                std::string name = entry.path().filename().string();
                std::transform(name.begin(),
                               name.end(),
                               name.begin(),
                               [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                if (name.find("gebco") != std::string::npos &&
                    entry.path().extension() == ".asc") {
                    return QString::fromStdString(entry.path().string());
                }
            }
        }
        return QString::fromStdString(worldlayers::resolveInputPath("dataset/osm_bathymetry_istanbul_genoa.geojson"));
    }

    static QString osmWorldDetailedLandPath() {
        return QString::fromStdString(
            worldlayers::resolveInputPath(".tmp_osm_land/land-polygons-split-4326/land-polygons-split-4326/land_polygons.shp"));
    }

    static QString defaultLandDatasetPath() {
        const std::string osm_local = worldlayers::resolveInputPath("dataset/osm_land_istanbul_genoa.geojson");
        if (std::filesystem::exists(osm_local)) {
            return QString::fromStdString(osm_local);
        }
        if (std::filesystem::exists(osmWorldDetailedLandPath().toStdString())) {
            return osmWorldDetailedLandPath();
        }
        return QString::fromStdString(worldlayers::resolveInputPath("dataset/ne_10m_land.geojson"));
    }

    static QString preferredNavigationSvgPath() {
        const QString preferred = QString::fromStdString(worldlayers::resolveOutputPath("output/mesh.svg"));
        if (QFileInfo::exists(preferred)) {
            return preferred;
        }
        const QString legacy = QString::fromStdString(worldlayers::resolveOutputPath("output/coastline_dp_mesh.svg"));
        if (QFileInfo::exists(legacy)) {
            return legacy;
        }
        return preferred;
    }

    static QString preferredNavigationCachePath() {
        const QString preferred = QString::fromStdString(worldlayers::resolveOutputPath("cache/triangulation_mesh.bin"));
        if (QFileInfo::exists(preferred)) {
            return preferred;
        }
        const QString legacy =
            QString::fromStdString(worldlayers::resolveOutputPath("cache/triangulation_coastline_dp_mesh.bin"));
        if (QFileInfo::exists(legacy)) {
            return legacy;
        }
        return preferred;
    }

    static std::vector<std::pair<QString, QString>> defaultBufferDatasetOptions() {
        const std::vector<std::pair<QString, QString>> options = {
            {"50 m buffer",
             QString::fromStdString(worldlayers::resolveInputPath(
                 "dataset/buffered/osm_land_istanbul_genoa_buffer_050m.geojson"))},
            {"100 m buffer",
             QString::fromStdString(worldlayers::resolveInputPath(
                 "dataset/buffered/osm_land_istanbul_genoa_buffer_100m.geojson"))},
            {"150 m buffer",
             QString::fromStdString(worldlayers::resolveInputPath(
                 "dataset/buffered/osm_land_istanbul_genoa_buffer_150m.geojson"))},
            {"200 m buffer",
             QString::fromStdString(worldlayers::resolveInputPath(
                 "dataset/buffered/osm_land_istanbul_genoa_buffer_200m.geojson"))},
        };
        return options;
    }

    void changeEvent(QEvent* event) override {
        QWidget::changeEvent(event);
        if (event->type() == QEvent::WindowStateChange && layout() != nullptr) {
            layout()->activate();
            updateGeometry();
        }
    }

    void buildUi() {
        auto* root = new QHBoxLayout(this);

        auto* left_panel = new QWidget(this);
        left_panel->setFixedWidth(420);
        auto* left_layout = new QVBoxLayout(left_panel);
        auto* tabs = new QTabWidget(left_panel);
        left_layout->addWidget(tabs);

        auto* dataset_tab_scroll = new QScrollArea(tabs);
        dataset_tab_scroll->setWidgetResizable(true);
        dataset_tab_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        auto* dataset_tab = new QWidget();
        auto* dataset_tab_layout = new QVBoxLayout(dataset_tab);

        auto* title = new QLabel("World Layers Desktop", dataset_tab);
        QFont title_font = title->font();
        title_font.setPointSize(16);
        title_font.setBold(true);
        title->setFont(title_font);
        dataset_tab_layout->addWidget(title);

        auto* subtitle =
            new QLabel("Load world land polygons, TSS, and bathymetry layers, render SVG, and preview the result.",
                       dataset_tab);
        subtitle->setWordWrap(true);
        dataset_tab_layout->addWidget(subtitle);

        auto* dataset_box = new QGroupBox("Land Dataset", dataset_tab);
        auto* dataset_layout = new QVBoxLayout(dataset_box);
        dataset_combo_ = new QComboBox(dataset_box);
        dataset_combo_->addItem("OSM world detailed", osmWorldDetailedLandPath());
        dataset_combo_->addItem("OSM detailed local",
                                QString::fromStdString(worldlayers::resolveInputPath("dataset/osm_land_istanbul_genoa.geojson")));
        dataset_combo_->addItem("Natural Earth 10m",
                                QString::fromStdString(worldlayers::resolveInputPath("dataset/ne_10m_land.geojson")));
        dataset_combo_->addItem("Natural Earth 50m",
                                QString::fromStdString(worldlayers::resolveInputPath("dataset/ne_50m_land.geojson")));
        dataset_combo_->addItem("Natural Earth 110m",
                                QString::fromStdString(worldlayers::resolveInputPath("dataset/ne_110m_land.geojson")));
        dataset_combo_->addItem("Custom path", "");
        dataset_layout->addWidget(dataset_combo_);
        dataset_layout->addLayout(makePathRow("Land GeoJSON", land_path_edit_, land_browse_button_));
        dataset_tab_layout->addWidget(dataset_box);

        auto* tss_box = new QGroupBox("TSS Layer", dataset_tab);
        auto* tss_layout = new QVBoxLayout(tss_box);
        tss_enabled_checkbox_ = new QCheckBox("Enable TSS layer", tss_box);
        tss_enabled_checkbox_->setChecked(true);
        tss_layout->addWidget(tss_enabled_checkbox_);
        tss_layout->addLayout(makePathRow("TSS GeoJSON", tss_path_edit_, tss_browse_button_));
        dataset_tab_layout->addWidget(tss_box);

        auto* bathy_box = new QGroupBox("Bathymetry Layer", dataset_tab);
        auto* bathy_layout = new QVBoxLayout(bathy_box);
        bathymetry_enabled_checkbox_ = new QCheckBox("Enable bathymetry layer", bathy_box);
        bathymetry_enabled_checkbox_->setChecked(true);
        bathy_layout->addWidget(bathymetry_enabled_checkbox_);
        bathy_layout->addLayout(makePathRow("Bathymetry GeoJSON", bathymetry_path_edit_, bathymetry_browse_button_));
        dataset_tab_layout->addWidget(bathy_box);

        auto* buffer_box = new QGroupBox("Coast Offset Zones", dataset_tab);
        auto* buffer_layout = new QVBoxLayout(buffer_box);
        buffer_zones_enabled_checkbox_ = new QCheckBox("Enable coast offset zones", buffer_box);
        buffer_layout->addWidget(buffer_zones_enabled_checkbox_);
        buffer_dataset_combo_ = new QComboBox(buffer_box);
        for (const auto& option : defaultBufferDatasetOptions()) {
            buffer_dataset_combo_->addItem(option.first, option.second);
        }
        buffer_dataset_combo_->addItem("Custom path", "");
        buffer_layout->addWidget(buffer_dataset_combo_);
        buffer_layout->addLayout(makePathRow("Buffer GeoJSON", buffer_dataset_path_edit_, buffer_dataset_browse_button_));
        auto* buffer_note = new QLabel(
            "The offset layer is selected from prebuilt 50m / 100m / 150m / 200m buffer datasets. Runtime buffering is not computed.",
            buffer_box);
        buffer_note->setWordWrap(true);
        buffer_layout->addWidget(buffer_note);
        dataset_tab_layout->addWidget(buffer_box);

        auto* viewport_box = new QGroupBox("Viewport", dataset_tab);
        auto* viewport_layout = new QFormLayout(viewport_box);
        custom_bounds_checkbox_ = new QCheckBox("Use custom viewport bounds", viewport_box);
        viewport_layout->addRow(custom_bounds_checkbox_);
        min_lat_spin_ = makeCoordSpinBox(-90.0, 90.0);
        max_lat_spin_ = makeCoordSpinBox(-90.0, 90.0);
        min_lon_spin_ = makeCoordSpinBox(-180.0, 180.0);
        max_lon_spin_ = makeCoordSpinBox(-180.0, 180.0);
        viewport_layout->addRow("Min lat", min_lat_spin_);
        viewport_layout->addRow("Max lat", max_lat_spin_);
        viewport_layout->addRow("Min lon", min_lon_spin_);
        viewport_layout->addRow("Max lon", max_lon_spin_);
        dataset_tab_layout->addWidget(viewport_box);

        auto* output_box = new QGroupBox("Output", dataset_tab);
        auto* output_layout = new QVBoxLayout(output_box);
        output_layout->addLayout(makePathRow("SVG output", svg_output_edit_, svg_browse_button_, true));
        output_layout->addLayout(makePathRow("Land + Mesh SVG", navigation_svg_edit_, navigation_svg_browse_button_));
        auto* navigation_button_row = new QHBoxLayout();
        load_navigation_svg_button_ = new QPushButton("Load Land + Mesh SVG", output_box);
        navigation_button_row->addStretch(1);
        navigation_button_row->addWidget(load_navigation_svg_button_);
        output_layout->addLayout(navigation_button_row);
        dataset_tab_layout->addWidget(output_box);

        auto* button_row = new QHBoxLayout();
        render_button_ = new QPushButton("Render", dataset_tab);
        fit_button_ = new QPushButton("Fit Preview", dataset_tab);
        button_row->addWidget(render_button_);
        button_row->addWidget(fit_button_);
        dataset_tab_layout->addLayout(button_row);

        loading_label_ = new QLabel("Ready", dataset_tab);
        loading_progress_ = new QProgressBar(dataset_tab);
        loading_progress_->setRange(0, 0);
        loading_progress_->setVisible(false);
        dataset_tab_layout->addWidget(loading_label_);
        dataset_tab_layout->addWidget(loading_progress_);

        status_box_ = new QPlainTextEdit(dataset_tab);
        status_box_->setReadOnly(true);
        status_box_->setMinimumHeight(180);
        dataset_tab_layout->addWidget(status_box_, 1);

        auto* help =
            new QLabel("Mouse wheel: zoom | Mouse drag: pan | Ctrl + Left Drag: box zoom\nRight click: previous zoom level",
                       dataset_tab);
        help->setWordWrap(true);
        dataset_tab_layout->addWidget(help);

        auto* route_tab_scroll = new QScrollArea(tabs);
        route_tab_scroll->setWidgetResizable(true);
        route_tab_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        auto* route_tab = new QWidget();
        auto* route_layout = new QVBoxLayout(route_tab);
        auto* route_title = new QLabel("Route Planner", route_tab);
        QFont route_title_font = route_title->font();
        route_title_font.setPointSize(16);
        route_title_font.setBold(true);
        route_title->setFont(route_title_font);
        route_layout->addWidget(route_title);

        auto* route_subtitle =
            new QLabel("Enter the start and goal coordinates. This tab prepares route-planning inputs.",
                       route_tab);
        route_subtitle->setWordWrap(true);
        route_layout->addWidget(route_subtitle);

        auto* start_box = new QGroupBox("Start Coordinate", route_tab);
        auto* start_layout = new QFormLayout(start_box);
        route_start_lat_spin_ = makeCoordSpinBox(-90.0, 90.0);
        route_start_lon_spin_ = makeCoordSpinBox(-180.0, 180.0);
        start_layout->addRow("Start lat", route_start_lat_spin_);
        start_layout->addRow("Start lon", route_start_lon_spin_);
        route_layout->addWidget(start_box);

        auto* goal_box = new QGroupBox("Goal Coordinate", route_tab);
        auto* goal_layout = new QFormLayout(goal_box);
        route_goal_lat_spin_ = makeCoordSpinBox(-90.0, 90.0);
        route_goal_lon_spin_ = makeCoordSpinBox(-180.0, 180.0);
        goal_layout->addRow("Goal lat", route_goal_lat_spin_);
        goal_layout->addRow("Goal lon", route_goal_lon_spin_);
        route_layout->addWidget(goal_box);

        auto* route_note =
            new QLabel("To pick the start and goal on the map, first render a layer map in the Dataset tab, then use the pick buttons below.",
                       route_tab);
        route_note->setWordWrap(true);
        route_layout->addWidget(route_note);
        route_tss_checkbox_ = new QCheckBox("Use TSS-guided routing", route_tab);
        route_tss_checkbox_->setChecked(true);
        route_tss_checkbox_->setToolTip("If disabled, the route is computed with the classic mesh shortcut.");
        route_layout->addWidget(route_tss_checkbox_);
        auto* route_pick_row = new QHBoxLayout();
        pick_start_button_ = new QPushButton("Pick Start on Map", route_tab);
        pick_goal_button_ = new QPushButton("Pick Goal on Map", route_tab);
        cancel_pick_button_ = new QPushButton("Cancel Pick", route_tab);
        plan_route_button_ = new QPushButton("Plan Route", route_tab);
        route_pick_row->addWidget(pick_start_button_);
        route_pick_row->addWidget(pick_goal_button_);
        route_pick_row->addWidget(cancel_pick_button_);
        route_pick_row->addWidget(plan_route_button_);
        route_layout->addLayout(route_pick_row);
        pick_status_label_ = new QLabel("Pick mode: off", route_tab);
        route_layout->addWidget(pick_status_label_);
        route_layout->addStretch(1);

        dataset_tab_scroll->setWidget(dataset_tab);
        route_tab_scroll->setWidget(route_tab);
        tabs->addTab(dataset_tab_scroll, "Dataset");
        tabs->addTab(route_tab_scroll, "Route Planner");

        root->addWidget(left_panel);

        scene_ = new QGraphicsScene(this);
        preview_view_ = new ZoomGraphicsView(this);
        preview_view_->setScene(scene_);
        preview_view_->setBackgroundBrush(QColor("#dfe7ef"));
        preview_view_->setCtrlDragZoomHandler([this](const QRectF& scene_rect) { zoomToSceneRect(scene_rect); });
        preview_view_->setRightClickHandler([this]() { zoomOutOneStep(); });
        preview_view_->setPointPickHandler([this](const QPointF& scene_pos) { handlePreviewPointPick(scene_pos); });
        root->addWidget(preview_view_, 1);

        connect(dataset_combo_,
                qOverload<int>(&QComboBox::currentIndexChanged),
                this,
                [this](int index) { onDatasetPresetChanged(index); });
        connect(land_browse_button_, &QPushButton::clicked, this, [this]() { browseOpenFile(land_path_edit_); });
        connect(tss_browse_button_, &QPushButton::clicked, this, [this]() { browseOpenFile(tss_path_edit_); });
        connect(bathymetry_browse_button_,
                &QPushButton::clicked,
                this,
                [this]() { browseOpenFile(bathymetry_path_edit_); });
        connect(buffer_dataset_combo_,
                qOverload<int>(&QComboBox::currentIndexChanged),
                this,
                [this](int index) { onBufferDatasetPresetChanged(index); });
        connect(buffer_dataset_browse_button_, &QPushButton::clicked, this, [this]() { browseOpenFile(buffer_dataset_path_edit_); });
        connect(svg_browse_button_, &QPushButton::clicked, this, [this]() { browseSaveFile(svg_output_edit_); });
        connect(navigation_svg_browse_button_,
                &QPushButton::clicked,
                this,
                [this]() { browseOpenSvgFile(navigation_svg_edit_); });
        connect(render_button_, &QPushButton::clicked, this, [this]() { renderNow(); });
        connect(fit_button_, &QPushButton::clicked, this, [this]() { fitPreview(); });
        connect(load_navigation_svg_button_, &QPushButton::clicked, this, [this]() { loadNavigationSvg(); });
        connect(pick_start_button_, &QPushButton::clicked, this, [this]() { beginPickMode(PickMode::Start); });
        connect(pick_goal_button_, &QPushButton::clicked, this, [this]() { beginPickMode(PickMode::Goal); });
        connect(cancel_pick_button_, &QPushButton::clicked, this, [this]() { clearPickMode(); });
        connect(plan_route_button_, &QPushButton::clicked, this, [this]() { planRoute(); });
        connect(custom_bounds_checkbox_, &QCheckBox::toggled, this, [this](bool checked) { setBoundsEnabled(checked); });
        connect(tss_enabled_checkbox_, &QCheckBox::toggled, this, [this](bool checked) { setLayerEnabled(tss_path_edit_, tss_browse_button_, checked); });
        connect(bathymetry_enabled_checkbox_,
                &QCheckBox::toggled,
                this,
                [this](bool checked) { setLayerEnabled(bathymetry_path_edit_, bathymetry_browse_button_, checked); });
        connect(buffer_zones_enabled_checkbox_,
                &QCheckBox::toggled,
                this,
                [this](bool checked) { setBufferZonesEnabled(checked); });
        connect(&render_watcher_, &QFutureWatcher<RenderTaskResult>::finished, this, [this]() { handleRenderFinished(); });
        connect(&route_watcher_,
                &QFutureWatcher<RoutePlanningTaskResult>::finished,
                this,
                [this]() { handleRoutePlanningFinished(); });
        auto* fullscreen_shortcut = new QShortcut(QKeySequence(Qt::Key_F11), this);
        connect(fullscreen_shortcut, &QShortcut::activated, this, [this]() { toggleFullScreen(); });
        auto* exit_fullscreen_shortcut = new QShortcut(QKeySequence(Qt::Key_Escape), this);
        connect(exit_fullscreen_shortcut, &QShortcut::activated, this, [this]() {
            if (isFullScreen()) {
                showNormal();
            }
        });
    }

    void loadDefaults() {
        const QString default_land = defaultLandDatasetPath();
        land_path_edit_->setText(default_land);
        const int preset_index = dataset_combo_->findData(default_land);
        dataset_combo_->setCurrentIndex(preset_index >= 0 ? preset_index : 4);
        tss_path_edit_->setText(QString::fromStdString(worldlayers::resolveInputPath("dataset/osm_tss_istanbul_genoa.geojson")));
        bathymetry_path_edit_->setText(defaultBathymetryDatasetPath());
        svg_output_edit_->setText(QString::fromStdString(worldlayers::resolveOutputPath("output/world_layers_desktop.svg")));
        navigation_svg_edit_->setText(preferredNavigationSvgPath());
        route_cache_path_ = preferredNavigationCachePath();
        if (buffer_dataset_combo_->count() > 0) {
            buffer_dataset_combo_->setCurrentIndex(0);
            buffer_dataset_path_edit_->setText(buffer_dataset_combo_->currentData().toString());
        }
        route_start_lat_spin_->setValue(41.0082);
        route_start_lon_spin_->setValue(28.9784);
        route_goal_lat_spin_->setValue(44.4056);
        route_goal_lon_spin_->setValue(8.9463);

        min_lat_spin_->setValue(34.0);
        max_lat_spin_->setValue(51.0);
        min_lon_spin_->setValue(0.0);
        max_lon_spin_->setValue(35.0);
        setBoundsEnabled(false);
        setLayerEnabled(tss_path_edit_, tss_browse_button_, true);
        setLayerEnabled(bathymetry_path_edit_, bathymetry_browse_button_, true);
        setBufferZonesEnabled(false);
        clearPickMode();

        if (default_land.contains("osm_land_istanbul_genoa") || default_land.endsWith("land_polygons.shp")) {
            custom_bounds_checkbox_->setChecked(true);
            setBoundsEnabled(true);
        }
    }

    QHBoxLayout* makePathRow(const QString& label,
                             QLineEdit*& edit_out,
                             QPushButton*& button_out,
                             bool save_dialog = false) {
        auto* row = new QHBoxLayout();
        auto* label_widget = new QLabel(label, this);
        label_widget->setMinimumWidth(110);
        edit_out = new QLineEdit(this);
        button_out = new QPushButton(save_dialog ? "Save As" : "Browse", this);
        row->addWidget(label_widget);
        row->addWidget(edit_out, 1);
        row->addWidget(button_out);
        return row;
    }

    QDoubleSpinBox* makeCoordSpinBox(double min_value, double max_value) {
        auto* spin = new QDoubleSpinBox(this);
        spin->setRange(min_value, max_value);
        spin->setDecimals(6);
        spin->setSingleStep(0.5);
        return spin;
    }

    void setBoundsEnabled(bool enabled) {
        min_lat_spin_->setEnabled(enabled);
        max_lat_spin_->setEnabled(enabled);
        min_lon_spin_->setEnabled(enabled);
        max_lon_spin_->setEnabled(enabled);
    }

    void setLayerEnabled(QLineEdit* edit, QPushButton* button, bool enabled) {
        edit->setEnabled(enabled);
        button->setEnabled(enabled);
    }

    void setBufferZonesEnabled(bool enabled) {
        buffer_dataset_combo_->setEnabled(enabled);
        buffer_dataset_path_edit_->setEnabled(enabled);
        buffer_dataset_browse_button_->setEnabled(enabled);
    }

    void onBufferDatasetPresetChanged(int index) {
        const QString path = buffer_dataset_combo_->itemData(index).toString();
        const bool is_custom = path.isEmpty();
        buffer_dataset_path_edit_->setEnabled(true);
        buffer_dataset_browse_button_->setEnabled(true);
        if (!is_custom) {
            buffer_dataset_path_edit_->setText(path);
        }
    }

    void onDatasetPresetChanged(int index) {
        const QString path = dataset_combo_->itemData(index).toString();
        const bool is_custom = path.isEmpty();
        land_path_edit_->setEnabled(true);
        land_browse_button_->setEnabled(true);
        if (!is_custom) {
            land_path_edit_->setText(path);
        }
        if (path == QString::fromStdString(worldlayers::resolveInputPath("dataset/osm_land_istanbul_genoa.geojson"))) {
            custom_bounds_checkbox_->setChecked(true);
            setBoundsEnabled(true);
            min_lat_spin_->setValue(34.0);
            max_lat_spin_->setValue(45.0);
            min_lon_spin_->setValue(8.9);
            max_lon_spin_->setValue(29.1);
        }
    }

    void applyRoutePresetForCurrentNavigationSvg(double min_lat, double max_lat, double min_lon, double max_lon) {
        const QString svg_path = navigation_svg_edit_ ? navigation_svg_edit_->text().trimmed() : QString();
        const QString stem = QFileInfo(svg_path).completeBaseName();
        auto pointInside = [&](double lat, double lon) {
            return lat >= min_lat && lat <= max_lat && lon >= min_lon && lon <= max_lon;
        };
        const bool current_inside =
            pointInside(route_start_lat_spin_->value(), route_start_lon_spin_->value()) &&
            pointInside(route_goal_lat_spin_->value(), route_goal_lon_spin_->value());
        if (current_inside) {
            return;
        }

        if (stem == "bosphorus_mesh") {
            route_start_lat_spin_->setValue(41.235000);
            route_start_lon_spin_->setValue(29.128000);
            route_goal_lat_spin_->setValue(40.965000);
            route_goal_lon_spin_->setValue(28.995000);
            refreshRouteMarkers();
            appendStatus("Route preset applied: Bosphorus start/goal.");
        }
    }

    void browseOpenFile(QLineEdit* target) {
        const QString file_name = QFileDialog::getOpenFileName(
            this,
            "Select data file",
            target->text(),
            "Supported (*.geojson *.json *.shp *.asc *.ascii);;All files (*)");
        if (!file_name.isEmpty()) {
            target->setText(file_name);
            if (target == land_path_edit_) {
                dataset_combo_->setCurrentIndex(dataset_combo_->count() - 1);
            } else if (target == buffer_dataset_path_edit_) {
                buffer_dataset_combo_->setCurrentIndex(buffer_dataset_combo_->count() - 1);
            }
        }
    }

    void browseOpenSvgFile(QLineEdit* target) {
        const QString file_name = QFileDialog::getOpenFileName(
            this,
            "Select SVG",
            target->text(),
            "SVG (*.svg);;All files (*)");
        if (!file_name.isEmpty()) {
            target->setText(file_name);
        }
    }

    void browseOpenCacheFile(QLineEdit* target) {
        const QString file_name = QFileDialog::getOpenFileName(
            this,
            "Select triangulation cache",
            target->text(),
            "Planner Cache (*.bin);;All files (*)");
        if (!file_name.isEmpty()) {
            target->setText(file_name);
        }
    }

    void browseSaveFile(QLineEdit* target) {
        const QString file_name = QFileDialog::getSaveFileName(
            this,
            "SVG output file",
            target->text(),
            "SVG (*.svg)");
        if (!file_name.isEmpty()) {
            target->setText(file_name);
        }
    }

    void setDatasetPathFromResolved(const QString& resolved_path) {
        land_path_edit_->setText(resolved_path);
        const int preset_index = dataset_combo_->findData(resolved_path);
        dataset_combo_->setCurrentIndex(preset_index >= 0 ? preset_index : dataset_combo_->count() - 1);
    }

    QString currentRouteCachePath() const {
        const QString svg_path = navigation_svg_edit_ ? navigation_svg_edit_->text().trimmed() : QString();
        const QString stem = QFileInfo(svg_path).completeBaseName();
        if (stem == "bosphorus_mesh") {
            return QString::fromStdString(worldlayers::resolveOutputPath("cache/triangulation_bosphorus_mesh.bin"));
        }
        if (stem == "mesh") {
            return QString::fromStdString(worldlayers::resolveOutputPath("cache/triangulation_mesh.bin"));
        }
        if (stem == "coastline_dp_mesh") {
            return QString::fromStdString(worldlayers::resolveOutputPath("cache/triangulation_coastline_dp_mesh.bin"));
        }
        if (stem == "marmara_straits_mesh") {
            return QString::fromStdString(
                worldlayers::resolveOutputPath("cache/triangulationdataset_marmara_straits_bathy_g01_c050.bin"));
        }
        if (stem == "marmara_straits_coastline_polygon") {
            return QString::fromStdString(worldlayers::resolveOutputPath("cache/triangulation_coastline_polygon.bin"));
        }
        return route_cache_path_;
    }

    void toggleFullScreen() {
        if (isFullScreen()) {
            showNormal();
            return;
        }
        showFullScreen();
    }

    QString preferredPreviewSvgPath(const QString& fallback_svg_path) const {
        const QString mesh_svg_path = navigation_svg_edit_ ? navigation_svg_edit_->text().trimmed() : QString();
        if (!mesh_svg_path.isEmpty() && QFileInfo::exists(mesh_svg_path)) {
            return mesh_svg_path;
        }
        return fallback_svg_path;
    }

    bool syncDatasetsFromCache(bool show_message_on_failure) {
        const QString cache_path = currentRouteCachePath();
        if (cache_path.isEmpty()) {
            if (show_message_on_failure) {
                QMessageBox::warning(this, "Missing file", "The cache path for route planning could not be determined.");
            }
            return false;
        }
        if (!QFileInfo::exists(cache_path)) {
            if (show_message_on_failure) {
                QMessageBox::warning(this, "File not found", "No suitable triangulation cache was found for Land + Mesh SVG.");
            }
            return false;
        }

        planner::PlannerCacheData cache;
        if (!planner::loadPlannerCache(cache_path.toStdString(), cache)) {
            if (show_message_on_failure) {
                QMessageBox::warning(this, "Cache error", "Failed to read triangulation cache.");
            }
            return false;
        }

        const bool has_embedded_land = !cache.land_polygons.empty();
        const QString resolved_land = QString::fromStdString(worldlayers::resolveInputPath(cache.dataset_path));
        const QString resolved_bathymetry =
            QString::fromStdString(worldlayers::resolveInputPath(cache.bathymetry_path));

        bool changed = false;
        if (!has_embedded_land && !resolved_land.isEmpty() && land_path_edit_->text().trimmed() != resolved_land) {
            setDatasetPathFromResolved(resolved_land);
            changed = true;
            appendStatus("Cache dataset sync: land dataset updated -> " + resolved_land);
        } else if (has_embedded_land) {
            appendStatus("Cache dataset sync: detailed land polygons will be used from the cache.");
        }

        if (!cache.bathymetry_path.empty()) {
            if (!bathymetry_enabled_checkbox_->isChecked()) {
                bathymetry_enabled_checkbox_->setChecked(true);
                changed = true;
            }
            if (bathymetry_path_edit_->text().trimmed() != resolved_bathymetry) {
                bathymetry_path_edit_->setText(resolved_bathymetry);
                changed = true;
                appendStatus("Cache dataset sync: bathymetry dataset updated -> " + resolved_bathymetry);
            }
        }

        if (changed && !current_render_land_dataset_.isEmpty() && current_render_land_dataset_ != resolved_land) {
            appendStatus("Warning: current map preview does not come from the same land dataset as the cache. Render again.");
        }
        if (changed && !resolved_bathymetry.isEmpty() &&
            !current_render_bathymetry_dataset_.isEmpty() &&
            current_render_bathymetry_dataset_ != resolved_bathymetry) {
            appendStatus("Warning: current map preview does not come from the same bathymetry dataset as the cache. Render again.");
        }
        return true;
    }

    worldlayers::Config gatherConfig() const {
        worldlayers::Config cfg;
        cfg.land_geojson = land_path_edit_->text().toStdString();
        cfg.tss_geojson = tss_enabled_checkbox_->isChecked() ? tss_path_edit_->text().toStdString() : "";
        cfg.bathymetry_geojson =
            bathymetry_enabled_checkbox_->isChecked() ? bathymetry_path_edit_->text().toStdString() : "";
        cfg.svg_file = svg_output_edit_->text().toStdString();
        cfg.allow_missing_tss = false;
        cfg.allow_missing_bathymetry = false;
        cfg.allow_missing_buffer_zones = false;
        cfg.enable_buffer_zones = buffer_zones_enabled_checkbox_->isChecked();
        cfg.buffer_zones_geojson = buffer_dataset_path_edit_->text().toStdString();
        cfg.bounds.enabled = custom_bounds_checkbox_->isChecked();
        cfg.bounds.min_lat = min_lat_spin_->value();
        cfg.bounds.max_lat = max_lat_spin_->value();
        cfg.bounds.min_lon = min_lon_spin_->value();
        cfg.bounds.max_lon = max_lon_spin_->value();
        return cfg;
    }

    void renderNow() {
        if (render_watcher_.isRunning() || triangulation_watcher_.isRunning() || route_watcher_.isRunning()) {
            return;
        }

        const auto cfg = gatherConfig();
        if (cfg.land_geojson.empty()) {
            QMessageBox::warning(this, "Missing data", "Land GeoJSON path cannot be empty.");
            return;
        }
        if (cfg.bounds.enabled &&
            (cfg.bounds.min_lat >= cfg.bounds.max_lat || cfg.bounds.min_lon >= cfg.bounds.max_lon)) {
            QMessageBox::warning(this, "Invalid viewport", "Check the min/max viewport values.");
            return;
        }
        if (cfg.enable_buffer_zones && cfg.buffer_zones_geojson.empty()) {
            QMessageBox::warning(this, "Missing dataset", "Select an offset dataset.");
            return;
        }
        setRenderBusy(true, "Loading map and generating SVG...");
        appendStatus("Loading started...");

        render_watcher_.setFuture(QtConcurrent::run([cfg]() {
            RenderTaskResult result;
            result.config = cfg;

            worldlayers::LoadedData data;
            if (!worldlayers::loadData(cfg, data, result.error_message, result.warning_message)) {
                return result;
            }
            if (!worldlayers::renderSvg(cfg, data, result.error_message)) {
                return result;
            }

            result.success = true;
            result.summary = worldlayers::summarize(data);
            result.resolved_svg_path = worldlayers::resolveOutputPath(cfg.svg_file);
            const RenderTaskResult transform = makeGeoTransformResult(data);
            result.has_geo_transform = transform.has_geo_transform;
            result.min_lon = transform.min_lon;
            result.max_lon = transform.max_lon;
            result.min_lat = transform.min_lat;
            result.max_lat = transform.max_lat;
            result.plot_x = transform.plot_x;
            result.plot_y = transform.plot_y;
            result.plot_w = transform.plot_w;
            result.plot_h = transform.plot_h;
            result.canvas_w = transform.canvas_w;
            result.canvas_h = transform.canvas_h;
            return result;
        }));
    }

    void handleRenderFinished() {
        setRenderBusy(false, "Ready");
        const RenderTaskResult result = render_watcher_.result();
        if (!result.success) {
            const QString error = QString::fromStdString(result.error_message);
            QMessageBox::critical(this, "Load error", error);
            appendStatus("Error: " + error);
            return;
        }

        current_geo_transform_ = result;
        current_render_land_dataset_ = QString::fromStdString(worldlayers::resolveInputPath(result.config.land_geojson));
        current_render_bathymetry_dataset_ =
            result.config.bathymetry_geojson.empty()
                ? QString()
                : QString::fromStdString(worldlayers::resolveInputPath(result.config.bathymetry_geojson));
        current_svg_path_ = preferredPreviewSvgPath(QString::fromStdString(result.resolved_svg_path));
        loadPreview(current_svg_path_);

        QString status;
        status += "Render completed.\n";
        status += "Land dataset: " + QString::fromStdString(result.config.land_geojson) + "\n";
        status += "TSS dataset: " +
                  QString::fromStdString(result.config.tss_geojson.empty() ? "disabled" : result.config.tss_geojson) +
                  "\n";
        status += "Bathymetry dataset: " +
                  QString::fromStdString(result.config.bathymetry_geojson.empty() ? "disabled"
                                                                                  : result.config.bathymetry_geojson) +
                  "\n";
        if (result.config.enable_buffer_zones) {
            status += "Coast offset dataset: " + QString::fromStdString(result.config.buffer_zones_geojson) + "\n";
        } else {
            status += "Coast offset zones: disabled\n";
        }
        if (result.config.bounds.enabled) {
            status += QString("Viewport lat:[%1, %2] lon:[%3, %4]\n")
                          .arg(result.config.bounds.min_lat)
                          .arg(result.config.bounds.max_lat)
                          .arg(result.config.bounds.min_lon)
                          .arg(result.config.bounds.max_lon);
        } else {
            status += "Viewport: all loaded layers\n";
        }
        status += QString("Land polygon: %1\n").arg(result.summary.land_polygon_count);
        status += QString("TSS feature: %1\n").arg(result.summary.tss_feature_count);
        status += QString("Bathymetry feature: %1\n").arg(result.summary.bathymetry_feature_count);
        status += QString("Offset zone polygon: %1\n").arg(result.summary.buffer_zone_polygon_count);
        status += "SVG output: " + QString::fromStdString(result.resolved_svg_path) + "\n";
        if (current_svg_path_ != QString::fromStdString(result.resolved_svg_path)) {
            status += "Preview SVG: " + current_svg_path_ + "\n";
            status += "Preview mode: mesh visualization\n";
        }
        if (!result.warning_message.empty()) {
            status += "Warning:\n" + QString::fromStdString(result.warning_message);
        }
        status_box_->setPlainText(status);

        maybeCaptureStartupScreenshot();
    }

    void beginPickMode(PickMode mode) {
        if (!current_geo_transform_.has_geo_transform) {
            QMessageBox::warning(this,
                                 "No georeference",
                                 "To pick a point on the map, first render a layer map in the Dataset tab.");
            return;
        }
        pick_mode_ = mode;
        preview_view_->setPointPickEnabled(true);
        if (pick_mode_ == PickMode::Start) {
            pick_status_label_->setText("Pick mode: start");
            appendStatus("Pick mode active: select the start point on the map.");
        } else if (pick_mode_ == PickMode::Goal) {
            pick_status_label_->setText("Pick mode: goal");
            appendStatus("Pick mode active: select the goal point on the map.");
        }
    }

    void clearPickMode() {
        pick_mode_ = PickMode::None;
        if (preview_view_) {
            preview_view_->setPointPickEnabled(false);
        }
        if (pick_status_label_) {
            pick_status_label_->setText("Pick mode: off");
        }
    }

    void clearRouteMarkers() {
        if (start_marker_item_ != nullptr) {
            delete start_marker_item_;
            start_marker_item_ = nullptr;
        }
        if (goal_marker_item_ != nullptr) {
            delete goal_marker_item_;
            goal_marker_item_ = nullptr;
        }
    }

    void updateRouteMarker(QGraphicsPathItem*& marker,
                           double lat,
                           double lon,
                           const QColor& fill_color) {
        if (!current_geo_transform_.has_geo_transform || scene_ == nullptr) {
            return;
        }
        const QPointF pos = projectGeoToSvgPoint(planner::LatLon{lat, lon}, current_geo_transform_);
        if (marker == nullptr) {
            marker = new QGraphicsPathItem(makePinPath());
            QPen pen(QColor("#ffffff"));
            pen.setWidthF(1.4);
            pen.setCosmetic(true);
            marker->setPen(pen);
            marker->setBrush(fill_color);
            marker->setZValue(30.0);
            scene_->addItem(marker);
        } else {
            marker->setBrush(fill_color);
        }
        marker->setPos(pos);
    }

    void refreshRouteMarkers() {
        if (!current_geo_transform_.has_geo_transform || scene_ == nullptr || scene_->items().isEmpty()) {
            return;
        }
        updateRouteMarker(start_marker_item_,
                          route_start_lat_spin_->value(),
                          route_start_lon_spin_->value(),
                          QColor("#1f9d55"));
        updateRouteMarker(goal_marker_item_,
                          route_goal_lat_spin_->value(),
                          route_goal_lon_spin_->value(),
                          QColor("#d64545"));
    }

    void clearRouteOverlay() {
        if (route_overlay_item_ != nullptr) {
            delete route_overlay_item_;
            route_overlay_item_ = nullptr;
        }
    }

    void renderRouteOverlay(const std::vector<planner::LatLon>& route_polyline) {
        clearRouteOverlay();
        if (!current_geo_transform_.has_geo_transform || route_polyline.size() < 2 || scene_ == nullptr) {
            return;
        }

        QPainterPath path;
        const QPointF start = projectGeoToSvgPoint(route_polyline.front(), current_geo_transform_);
        path.moveTo(start);
        for (size_t i = 1; i < route_polyline.size(); ++i) {
            path.lineTo(projectGeoToSvgPoint(route_polyline[i], current_geo_transform_));
        }

        auto* item = new QGraphicsPathItem(path);
        QPen pen(QColor("#084c97"));
        pen.setWidthF(3.2);
        pen.setCosmetic(true);
        pen.setCapStyle(Qt::RoundCap);
        pen.setJoinStyle(Qt::RoundJoin);
        item->setPen(pen);
        item->setBrush(Qt::NoBrush);
        item->setZValue(28.0);
        scene_->addItem(item);
        route_overlay_item_ = item;
    }

    void planRoute() {
        if (render_watcher_.isRunning() || triangulation_watcher_.isRunning() || route_watcher_.isRunning()) {
            return;
        }
        if (!syncDatasetsFromCache(true)) {
            return;
        }

        const QString cache_path = currentRouteCachePath();
        if (cache_path.isEmpty()) {
            QMessageBox::warning(this, "Missing file", "Triangulation cache is required for route planning.");
            return;
        }
        if (!QFileInfo::exists(cache_path)) {
            QMessageBox::warning(this, "File not found", "Triangulation cache not found.");
            return;
        }

        clearPickMode();
        clearRouteOverlay();
        setRenderBusy(true, "Route planning in progress...");
        appendStatus("Route planning started in background...");

        const planner::LatLon requested_start{route_start_lat_spin_->value(), route_start_lon_spin_->value()};
        const planner::LatLon requested_goal{route_goal_lat_spin_->value(), route_goal_lon_spin_->value()};
        const auto overlay_cfg = gatherConfig();
        const bool use_tss_routing = route_tss_checkbox_ != nullptr && route_tss_checkbox_->isChecked();
        const std::string tss_geojson = use_tss_routing ? overlay_cfg.tss_geojson : std::string();
        planner::TrafficRoutingConfig traffic_cfg = defaultDesktopTrafficRoutingConfig();
        if (!use_tss_routing) {
            traffic_cfg.enable_tss_routing = false;
            traffic_cfg.hard_tss_corridor = false;
        }

        appendStatus(QString("Route planning mode: %1").arg(use_tss_routing ? "TSS enabled" : "TSS disabled"));

        route_watcher_.setFuture(QtConcurrent::run([cache_path, requested_start, requested_goal, tss_geojson, traffic_cfg]() {
            using namespace planner;

            RoutePlanningTaskResult result;

            PlannerCacheData cache;
            if (!loadPlannerCache(cache_path.toStdString(), cache)) {
                result.error_message = "Failed to read triangulation cache.";
                return result;
            }
            if (cache.vertices.empty() || cache.triangles.empty()) {
                result.error_message = "Triangulation cache is empty.";
                return result;
            }

            std::vector<LandPolygon> all_land;
            if (!loadLandForPlannerCache(cache, all_land, result.error_message)) {
                return result;
            }

            std::vector<BathymetryFeature> all_bathymetry;
            if (!loadBathymetryForPlannerCache(cache, all_bathymetry, result.error_message)) {
                return result;
            }

            const auto all_land_index = buildLandSpatialIndex(all_land, 0.5);
            const auto all_shallow_polygons = buildShallowWaterPolygons(all_bathymetry, cache.min_depth_m);
            const auto all_shallow_index = buildLandSpatialIndex(all_shallow_polygons, 0.5);
            const double clearance_km = cache.clearance_m / 1000.0;
            auto pointInsideBounds = [&](const LatLon& p) {
                return p.lat_deg >= cache.min_lat && p.lat_deg <= cache.max_lat &&
                       p.lon_deg >= cache.min_lon && p.lon_deg <= cache.max_lon;
            };
            if (!pointInsideBounds(requested_start) || !pointInsideBounds(requested_goal)) {
                result.error_message =
                    "Start or goal is outside the selected bounds. Computation only runs within the current mesh coverage.";
                return result;
            }

            result.start_geo = nearestNavigablePoint(requested_start,
                                                     all_land,
                                                     all_land_index,
                                                     clearance_km,
                                                     all_shallow_polygons,
                                                     all_shallow_index);
            result.goal_geo = nearestNavigablePoint(requested_goal,
                                                    all_land,
                                                    all_land_index,
                                                    clearance_km,
                                                    all_shallow_polygons,
                                                    all_shallow_index);

            if (!pointInsideBounds(result.start_geo) || !pointInsideBounds(result.goal_geo)) {
                result.error_message = "Start veya goal triangulation cache kapsami disinda.";
                return result;
            }

            auto land = filterPolygonsByBounds(all_land, cache.min_lat, cache.max_lat, cache.min_lon, cache.max_lon);
            auto land_index = buildLandSpatialIndex(land, 0.5);
            auto bathymetry =
                filterBathymetryByBounds(all_bathymetry, cache.min_lat, cache.max_lat, cache.min_lon, cache.max_lon);
            auto shallow_polygons = buildShallowWaterPolygons(bathymetry, cache.min_depth_m);
            auto shallow_index = buildLandSpatialIndex(shallow_polygons, 0.5);
            std::vector<TssFeature> tss;
            if (!tss_geojson.empty()) {
                const std::string resolved_tss = worldlayers::resolveInputPath(tss_geojson);
                if (!resolved_tss.empty()) {
                    std::vector<TssFeature> all_tss;
                    if (loadTssFeaturesGeoJson(resolved_tss, all_tss)) {
                        tss = filterTssByBounds(all_tss, cache.min_lat, cache.max_lat, cache.min_lon, cache.max_lon);
                    } else {
                        result.warning_message = "Failed to read TSS data; route was computed with classic cost.";
                    }
                }
            }

            const Projection proj = makeProjection(cache.min_lat, cache.max_lat, cache.min_lon, cache.max_lon);
            std::vector<XY> vertex_xy;
            vertex_xy.reserve(cache.vertices.size());
            for (const auto& v : cache.vertices) {
                vertex_xy.push_back(toXY(v.geo, proj));
            }
            const auto traffic_data =
                buildTriangleTrafficData(cache.triangles, proj, tss, result.start_geo, result.goal_geo, traffic_cfg);
            const auto tri_adj = buildTriangleAdjacency(cache.triangles);
            const auto vertex_usage = buildVertexUsage(cache.triangles, static_cast<int>(cache.vertices.size()));
            std::vector<int> vertex_traffic_usage(cache.vertices.size(), 0);
            if (traffic_cfg.hard_tss_corridor && traffic_data.active) {
                for (size_t tri_idx = 0; tri_idx < cache.triangles.size(); ++tri_idx) {
                    if (tri_idx >= traffic_data.triangle_in_hard_corridor.size() ||
                        traffic_data.triangle_in_hard_corridor[tri_idx] == 0) {
                        continue;
                    }
                    const auto& tri = cache.triangles[tri_idx];
                    vertex_traffic_usage[tri.a] += 1;
                    vertex_traffic_usage[tri.b] += 1;
                    vertex_traffic_usage[tri.c] += 1;
                }
            }
            const auto start_ranked = rankUsableVerticesByDistance(cache.vertices, vertex_usage, result.start_geo);
            const auto goal_ranked = rankUsableVerticesByDistance(cache.vertices, vertex_usage, result.goal_geo);
            if (start_ranked.empty() || goal_ranked.empty()) {
                result.error_message = "No usable start/goal vertex found on the mesh.";
                return result;
            }

            const auto vertex_comp =
                buildVertexComponentIds(cache.triangles, static_cast<int>(cache.vertices.size()), vertex_usage);
            constexpr int kMaxStartCandidates = 300;
            constexpr int kMaxGoalCandidates = 300;
            constexpr int kMaxGoalsPerComponent = 24;
            struct CandidatePair {
                int sv = -1;
                int gv = -1;
                double cost = std::numeric_limits<double>::infinity();
            };

            std::unordered_map<int, std::vector<std::pair<double, int>>> goals_by_comp;
            goals_by_comp.reserve(64);
            for (int i = 0; i < static_cast<int>(goal_ranked.size()) && i < kMaxGoalCandidates; ++i) {
                const auto& [dist, gv] = goal_ranked[i];
                if (!edgeNavigable(cache.vertices[gv].geo,
                                   result.goal_geo,
                                   land,
                                   land_index,
                                   clearance_km,
                                   shallow_polygons,
                                   shallow_index)) {
                    continue;
                }
                if (traffic_cfg.hard_tss_corridor && traffic_data.active &&
                    (gv < 0 || gv >= static_cast<int>(vertex_traffic_usage.size()) || vertex_traffic_usage[gv] <= 0)) {
                    continue;
                }
                const int comp = vertex_comp[gv];
                if (comp < 0) {
                    continue;
                }
                auto& bucket = goals_by_comp[comp];
                if (static_cast<int>(bucket.size()) < kMaxGoalsPerComponent) {
                    bucket.push_back({dist, gv});
                }
            }

            std::vector<CandidatePair> candidate_pairs;
            candidate_pairs.reserve(static_cast<std::size_t>(kMaxStartCandidates * 16));
            for (int i = 0; i < static_cast<int>(start_ranked.size()) && i < kMaxStartCandidates; ++i) {
                const auto& [start_dist, sv] = start_ranked[i];
                if (!edgeNavigable(result.start_geo,
                                   cache.vertices[sv].geo,
                                   land,
                                   land_index,
                                   clearance_km,
                                   shallow_polygons,
                                   shallow_index)) {
                    continue;
                }
                if (traffic_cfg.hard_tss_corridor && traffic_data.active &&
                    (sv < 0 || sv >= static_cast<int>(vertex_traffic_usage.size()) || vertex_traffic_usage[sv] <= 0)) {
                    continue;
                }
                const int comp = vertex_comp[sv];
                if (comp < 0) {
                    continue;
                }
                const auto it = goals_by_comp.find(comp);
                if (it == goals_by_comp.end()) {
                    continue;
                }
                for (const auto& [goal_dist, gv] : it->second) {
                    candidate_pairs.push_back({sv, gv, start_dist + goal_dist});
                }
            }
            std::sort(candidate_pairs.begin(), candidate_pairs.end(), [](const CandidatePair& a, const CandidatePair& b) {
                if (a.cost != b.cost) {
                    return a.cost < b.cost;
                }
                if (a.sv != b.sv) {
                    return a.sv < b.sv;
                }
                return a.gv < b.gv;
            });

            TriangleSearchResult tri_search;
            for (const auto& pair : candidate_pairs) {
                auto attempt =
                    searchTriangleSequence(cache.triangles, tri_adj, pair.sv, pair.gv, vertex_xy, &traffic_data);
                if (!attempt.found) {
                    continue;
                }
                result.mesh_start_vertex = pair.sv;
                result.mesh_goal_vertex = pair.gv;
                tri_search = std::move(attempt);
                break;
            }

            result.success = true;
            if (result.mesh_start_vertex < 0 || result.mesh_goal_vertex < 0) {
                if (traffic_cfg.hard_tss_corridor && traffic_data.active) {
                    result.warning_message =
                        "TSS hard corridor is active. The selected start/goal may not be in the same permitted traffic corridor.";
                }
                return result;
            }

            result.found = true;
            result.triangle_sequence = tri_search.triangle_sequence;
            std::vector<LatLon> raw_mesh_line;
            raw_mesh_line.reserve(tri_search.triangle_sequence.size() + 4);
            raw_mesh_line.push_back(result.start_geo);
            raw_mesh_line.push_back(cache.vertices[result.mesh_start_vertex].geo);
            for (int tri_id : tri_search.triangle_sequence) {
                raw_mesh_line.push_back(toGeo(cache.triangles[tri_id].centroid, proj));
            }
            raw_mesh_line.push_back(cache.vertices[result.mesh_goal_vertex].geo);
            raw_mesh_line.push_back(result.goal_geo);

            if (traffic_data.active) {
                result.route_polyline = buildRouteAlongTssCenterline(result.start_geo,
                                                                     result.goal_geo,
                                                                     proj,
                                                                     tss,
                                                                     land,
                                                                     land_index,
                                                                     clearance_km,
                                                                     shallow_polygons,
                                                                     shallow_index);
                if (!result.route_polyline.empty()) {
                    result.used_tss_centerline = true;
                } else if (traffic_cfg.hard_tss_corridor) {
                    result.route_polyline = shortcutPathWithTraffic(raw_mesh_line,
                                                                    proj,
                                                                    land,
                                                                    land_index,
                                                                    clearance_km,
                                                                    shallow_polygons,
                                                                    shallow_index,
                                                                    traffic_data);
                } else {
                    result.route_polyline = shortcutPath(raw_mesh_line,
                                                         land,
                                                         land_index,
                                                         clearance_km,
                                                         shallow_polygons,
                                                         shallow_index);
                }
            } else {
                result.route_polyline = shortcutPath(raw_mesh_line,
                                                     land,
                                                     land_index,
                                                     clearance_km,
                                                     shallow_polygons,
                                                     shallow_index);
            }
            if (result.route_polyline.size() < 2) {
                result.found = false;
                result.route_polyline.clear();
                return result;
            }
            result.total_km = polylineLengthKm(result.route_polyline);
            return result;
        }));
    }

    void handleRoutePlanningFinished() {
        setRenderBusy(false, "Ready");
        const RoutePlanningTaskResult result = route_watcher_.result();
        if (!result.success) {
            clearRouteOverlay();
            QMessageBox::warning(this, "Route planning", QString::fromStdString(result.error_message));
            appendStatus("Route planning error: " + QString::fromStdString(result.error_message));
            return;
        }

        route_start_lat_spin_->setValue(result.start_geo.lat_deg);
        route_start_lon_spin_->setValue(result.start_geo.lon_deg);
        route_goal_lat_spin_->setValue(result.goal_geo.lat_deg);
        route_goal_lon_spin_->setValue(result.goal_geo.lon_deg);
        refreshRouteMarkers();
        if (!result.warning_message.empty()) {
            appendStatus("Route planning warning: " + QString::fromStdString(result.warning_message));
        }

        if (!result.found) {
            clearRouteOverlay();
            if (!result.warning_message.empty()) {
                QMessageBox::warning(this, "Route planning", QString::fromStdString(result.warning_message));
            }
            appendStatus("Route planning result: no route found.");
            return;
        }

        renderRouteOverlay(result.route_polyline);
        appendStatus(QString("Route found: %1 km, polyline points=%2, triangle corridor=%3%4")
                         .arg(result.total_km, 0, 'f', 1)
                         .arg(static_cast<int>(result.route_polyline.size()))
                         .arg(static_cast<int>(result.triangle_sequence.size()))
                         .arg(result.used_tss_centerline ? ", TSS centerline" : ""));
    }

    void handlePreviewPointPick(const QPointF& scene_pos) {
        if (pick_mode_ == PickMode::None || !current_geo_transform_.has_geo_transform || scene_->items().isEmpty()) {
            return;
        }

        const QRectF scene_rect = scene_->sceneRect();
        if (!scene_rect.isValid() || scene_rect.isEmpty()) {
            return;
        }

        const double svg_x =
            ((scene_pos.x() - scene_rect.left()) / scene_rect.width()) * current_geo_transform_.canvas_w;
        const double svg_y =
            ((scene_pos.y() - scene_rect.top()) / scene_rect.height()) * current_geo_transform_.canvas_h;

        const double plot_min_x = current_geo_transform_.plot_x;
        const double plot_max_x = current_geo_transform_.plot_x + current_geo_transform_.plot_w;
        const double plot_min_y = current_geo_transform_.plot_y;
        const double plot_max_y = current_geo_transform_.plot_y + current_geo_transform_.plot_h;
        if (svg_x < plot_min_x || svg_x > plot_max_x || svg_y < plot_min_y || svg_y > plot_max_y) {
            appendStatus("Selection is outside the plot area, try again.");
            return;
        }

        const double lon_t = (svg_x - current_geo_transform_.plot_x) / current_geo_transform_.plot_w;
        const double lat_t = (svg_y - current_geo_transform_.plot_y) / current_geo_transform_.plot_h;
        const double lon =
            current_geo_transform_.min_lon + lon_t * (current_geo_transform_.max_lon - current_geo_transform_.min_lon);
        const double lat =
            current_geo_transform_.max_lat - lat_t * (current_geo_transform_.max_lat - current_geo_transform_.min_lat);

        if (pick_mode_ == PickMode::Start) {
            route_start_lat_spin_->setValue(lat);
            route_start_lon_spin_->setValue(lon);
            refreshRouteMarkers();
            appendStatus(QString("Start selected: lat=%1 lon=%2").arg(lat, 0, 'f', 6).arg(lon, 0, 'f', 6));
        } else if (pick_mode_ == PickMode::Goal) {
            route_goal_lat_spin_->setValue(lat);
            route_goal_lon_spin_->setValue(lon);
            refreshRouteMarkers();
            appendStatus(QString("Goal selected: lat=%1 lon=%2").arg(lat, 0, 'f', 6).arg(lon, 0, 'f', 6));
        }

        clearPickMode();
    }

    void loadNavigationSvg() {
        if (triangulation_watcher_.isRunning() || route_watcher_.isRunning()) {
            return;
        }
        syncDatasetsFromCache(false);
        const QString cache_path = currentRouteCachePath();
        const QString svg_path = navigation_svg_edit_->text().trimmed();
        const bool had_geo_transform = current_geo_transform_.has_geo_transform;
        if (svg_path.isEmpty()) {
            QMessageBox::warning(this, "Missing file", "Land + Mesh SVG path cannot be empty.");
            return;
        }

        planner::PlannerCacheData cache;
        const bool have_cache = !cache_path.isEmpty() && QFileInfo::exists(cache_path) &&
                                planner::loadPlannerCache(cache_path.toStdString(), cache);
        if (have_cache) {
            const auto overlay_cfg = gatherConfig();
            std::vector<planner::LandPolygon> land;
            std::string error_message;
            if (!loadLandForPlannerCache(cache, land, error_message)) {
                QMessageBox::warning(this, "Cache error", QString::fromStdString(error_message));
                return;
            }

            worldlayers::LoadedData overlay_data;
            worldlayers::Config layer_cfg = overlay_cfg;
            layer_cfg.land_geojson = cache.dataset_path.empty() ? overlay_cfg.land_geojson : cache.dataset_path;
            layer_cfg.bounds.enabled = true;
            layer_cfg.bounds.min_lat = cache.min_lat;
            layer_cfg.bounds.max_lat = cache.max_lat;
            layer_cfg.bounds.min_lon = cache.min_lon;
            layer_cfg.bounds.max_lon = cache.max_lon;
            if (!worldlayers::loadData(layer_cfg, overlay_data, error_message, error_message)) {
                QMessageBox::warning(this, "Layer error", QString::fromStdString(error_message));
                return;
            }

            std::vector<planner::BathymetryFeature> bathymetry;
            if (!loadBathymetryForPlannerCache(cache, bathymetry, error_message)) {
                QMessageBox::warning(this, "Cache error", QString::fromStdString(error_message));
                return;
            }
            bathymetry = planner::filterBathymetryByBounds(
                bathymetry, cache.min_lat, cache.max_lat, cache.min_lon, cache.max_lon);

            const std::filesystem::path output_path(svg_path.toStdString());
            if (output_path.has_parent_path()) {
                std::error_code ec;
                std::filesystem::create_directories(output_path.parent_path(), ec);
                if (ec) {
                    QMessageBox::warning(this,
                                         "SVG error",
                                         "Failed to create the Land + Mesh SVG directory: " +
                                             QString::fromStdString(output_path.parent_path().string()));
                    return;
                }
            }

            planner::writeSvgPlot(svg_path.toStdString(),
                                  cache.vertices,
                                  cache.triangles,
                                  {},
                                  {},
                                  land,
                                  overlay_data.tss,
                                  bathymetry,
                                  overlay_data.buffer_zones);
            worldlayers::LoadedData transform_data;
            transform_data.land = land;
            transform_data.tss = overlay_data.tss;
            transform_data.bathymetry = bathymetry;
            transform_data.buffer_zones = overlay_data.buffer_zones;
            current_geo_transform_ = makeGeoTransformResult(transform_data);
            appendStatus("Land + Mesh SVG regenerated from cache: " + svg_path);
        } else if (!QFileInfo::exists(svg_path)) {
            QMessageBox::warning(this, "File not found", "Land + Mesh SVG not found.");
            return;
        }

        current_svg_path_ = svg_path;
        loadPreview(current_svg_path_);
        if (!had_geo_transform && !have_cache) {
            current_geo_transform_ = RenderTaskResult{};
        }
        clearPickMode();
        if (current_geo_transform_.has_geo_transform) {
            applyRoutePresetForCurrentNavigationSvg(cache.min_lat, cache.max_lat, cache.min_lon, cache.max_lon);
            refreshRouteMarkers();
        }
        appendStatus("Land + Mesh SVG loaded: " + current_svg_path_);
        if (have_cache) {
            appendStatus("Georeference was built from cache bounds; start/goal picking on the map is available.");
        } else if (had_geo_transform) {
            appendStatus("Current georeference was preserved; start/goal picking on the map is available.");
        }
    }

    void clearTriangulationLayer() {
        if (triangulation_overlay_item_ != nullptr) {
            delete triangulation_overlay_item_;
            triangulation_overlay_item_ = nullptr;
        }
    }

    void loadTriangulationLayer() {
        if (render_watcher_.isRunning() || triangulation_watcher_.isRunning() || route_watcher_.isRunning()) {
            return;
        }
        if (!syncDatasetsFromCache(true)) {
            return;
        }
        if (!current_geo_transform_.has_geo_transform) {
            QMessageBox::warning(this,
                                 "No georeference",
                                 "To load the triangle layer, first render a layer map in the Dataset tab.");
            return;
        }

        const QString cache_path =
            triangulation_cache_edit_ && !triangulation_cache_edit_->text().trimmed().isEmpty()
                ? triangulation_cache_edit_->text().trimmed()
                : currentRouteCachePath();
        if (cache_path.isEmpty()) {
            QMessageBox::warning(this, "Missing file", "Triangulation cache path cannot be empty.");
            return;
        }
        if (!QFileInfo::exists(cache_path)) {
            QMessageBox::warning(this, "File not found", "Triangulation cache not found.");
            return;
        }

        clearPickMode();
        setRenderBusy(true, "Loading triangle layer...");
        appendStatus("Reading triangulation cache in background...");

        const RenderTaskResult transform = current_geo_transform_;
        triangulation_watcher_.setFuture(QtConcurrent::run([cache_path, transform]() {
            TriangulationLayerTaskResult result;

            planner::PlannerCacheData cache;
            if (!planner::loadPlannerCache(cache_path.toStdString(), cache)) {
                result.error_message = "Failed to read triangulation cache.";
                return result;
            }
            if (cache.vertices.empty() || cache.triangles.empty()) {
                result.error_message = "Triangulation cache contains no vertices or triangles.";
                return result;
            }

            std::unordered_set<std::uint64_t> drawn;
            drawn.reserve(cache.triangles.size() * 2ULL);
            QPainterPath path;

            for (const auto& tri : cache.triangles) {
                const std::pair<int, int> edges[3] = {{tri.a, tri.b}, {tri.b, tri.c}, {tri.c, tri.a}};
                for (const auto& edge : edges) {
                    if (edge.first < 0 || edge.second < 0 ||
                        edge.first >= static_cast<int>(cache.vertices.size()) ||
                        edge.second >= static_cast<int>(cache.vertices.size())) {
                        continue;
                    }
                    const auto key = desktopEdgeKey(edge.first, edge.second);
                    if (!drawn.insert(key).second) {
                        continue;
                    }
                    const QPointF a = projectGeoToSvgPoint(cache.vertices[edge.first].geo, transform);
                    const QPointF b = projectGeoToSvgPoint(cache.vertices[edge.second].geo, transform);
                    path.moveTo(a);
                    path.lineTo(b);
                }
            }

            if (path.isEmpty()) {
                result.error_message = "No drawable edges were produced from the triangulation cache.";
                return result;
            }

            result.success = true;
            result.path = path;
            result.triangle_count = static_cast<std::uint64_t>(cache.triangles.size());
            result.edge_count = static_cast<std::uint64_t>(drawn.size());
            return result;
        }));
    }

    void handleTriangulationLayerFinished() {
        setRenderBusy(false, "Ready");
        const TriangulationLayerTaskResult result = triangulation_watcher_.result();
        if (!result.success) {
            QMessageBox::warning(this, "Triangle layer", QString::fromStdString(result.error_message));
            appendStatus("Triangle layer error: " + QString::fromStdString(result.error_message));
            return;
        }

        clearTriangulationLayer();

        auto* item = new QGraphicsPathItem(result.path);
        QPen pen(QColor("#5c6f82"));
        pen.setWidthF(0.7);
        pen.setCosmetic(true);
        item->setPen(pen);
        item->setBrush(Qt::NoBrush);
        item->setZValue(20.0);
        scene_->addItem(item);
        triangulation_overlay_item_ = item;

        appendStatus(QString("Triangle layer loaded: %1 triangles, %2 unique edges")
                         .arg(static_cast<qulonglong>(result.triangle_count))
                         .arg(static_cast<qulonglong>(result.edge_count)));
    }

    void setRenderBusy(bool busy, const QString& message) {
        loading_label_->setText(message);
        loading_progress_->setVisible(busy);
        render_button_->setEnabled(!busy);
        fit_button_->setEnabled(!busy);
        load_navigation_svg_button_->setEnabled(!busy);
        pick_start_button_->setEnabled(!busy);
        pick_goal_button_->setEnabled(!busy);
        cancel_pick_button_->setEnabled(!busy);
        plan_route_button_->setEnabled(!busy);
        dataset_combo_->setEnabled(!busy);
        land_path_edit_->setEnabled(!busy);
        land_browse_button_->setEnabled(!busy);
        tss_enabled_checkbox_->setEnabled(!busy);
        bathymetry_enabled_checkbox_->setEnabled(!busy);
        buffer_zones_enabled_checkbox_->setEnabled(!busy);
        custom_bounds_checkbox_->setEnabled(!busy);
        setLayerEnabled(tss_path_edit_, tss_browse_button_, !busy && tss_enabled_checkbox_->isChecked());
        setLayerEnabled(bathymetry_path_edit_, bathymetry_browse_button_, !busy && bathymetry_enabled_checkbox_->isChecked());
        setBufferZonesEnabled(!busy && buffer_zones_enabled_checkbox_->isChecked());
        min_lat_spin_->setEnabled(!busy && custom_bounds_checkbox_->isChecked());
        max_lat_spin_->setEnabled(!busy && custom_bounds_checkbox_->isChecked());
        min_lon_spin_->setEnabled(!busy && custom_bounds_checkbox_->isChecked());
        max_lon_spin_->setEnabled(!busy && custom_bounds_checkbox_->isChecked());
        svg_output_edit_->setEnabled(!busy);
        svg_browse_button_->setEnabled(!busy);
        navigation_svg_edit_->setEnabled(!busy);
        navigation_svg_browse_button_->setEnabled(!busy);
        if (busy) {
            QApplication::setOverrideCursor(Qt::BusyCursor);
        } else {
            QApplication::restoreOverrideCursor();
        }
    }

    void loadPreview(const QString& svg_path) {
        scene_->clear();
        triangulation_overlay_item_ = nullptr;
        route_overlay_item_ = nullptr;
        start_marker_item_ = nullptr;
        goal_marker_item_ = nullptr;
        auto* item = new QGraphicsSvgItem(svg_path);
        if (!item->renderer() || !item->renderer()->isValid()) {
            delete item;
            QMessageBox::critical(this, "Preview error", "Failed to load SVG preview.");
            return;
        }
        scene_->addItem(item);
        scene_->setSceneRect(item->boundingRect());
        zoom_history_.clear();
        refreshRouteMarkers();
        fitPreview();
    }

    void fitPreview() {
        if (!scene_->items().isEmpty()) {
            preview_view_->fitInView(scene_->sceneRect(), Qt::KeepAspectRatio);
        }
    }

    QRectF currentVisibleSceneRect() const {
        if (!preview_view_ || scene_->items().isEmpty()) {
            return {};
        }
        QRectF rect = preview_view_->mapToScene(preview_view_->viewport()->rect()).boundingRect();
        rect = rect.intersected(scene_->sceneRect());
        return rect;
    }

    void zoomToSceneRect(const QRectF& scene_rect_selection) {
        if (scene_->items().isEmpty()) {
            return;
        }

        const QRectF scene_rect = scene_->sceneRect();
        QRectF target = scene_rect_selection.normalized().intersected(scene_rect);
        if (!target.isValid() || target.isEmpty()) {
            return;
        }

        const QRectF current_rect = currentVisibleSceneRect();
        if (current_rect.isValid() && !current_rect.isEmpty()) {
            zoom_history_.push_back(current_rect);
        }
        preview_view_->fitInView(target, Qt::KeepAspectRatio);
        appendStatus(QString("Ctrl+Drag zoom applied: w=%1 h=%2")
                         .arg(target.width(), 0, 'f', 1)
                         .arg(target.height(), 0, 'f', 1));
    }

    void zoomOutOneStep() {
        if (scene_->items().isEmpty()) {
            return;
        }
        if (zoom_history_.empty()) {
            fitPreview();
            appendStatus("Right click: returned to full view");
            return;
        }

        const QRectF previous = zoom_history_.back();
        zoom_history_.pop_back();
        if (!previous.isValid() || previous.isEmpty()) {
            fitPreview();
            return;
        }
        preview_view_->fitInView(previous, Qt::KeepAspectRatio);
        appendStatus("Right click: returned to previous zoom level");
    }

    void appendStatus(const QString& line) {
        const QString current = status_box_->toPlainText();
        status_box_->setPlainText(current.isEmpty() ? line : (current + "\n" + line));
    }

    void maybeCaptureStartupScreenshot() {
        if (startup_screenshot_path_.isEmpty() || startup_screenshot_done_) {
            return;
        }

        startup_screenshot_done_ = true;
        const QString screenshot_path = startup_screenshot_path_;
        QTimer::singleShot(250, this, [this, screenshot_path]() {
            std::error_code ec;
            const std::filesystem::path output_path(screenshot_path.toStdString());
            if (output_path.has_parent_path()) {
                std::filesystem::create_directories(output_path.parent_path(), ec);
            }

            const bool ok = grab().save(screenshot_path);
            if (!ok) {
                appendStatus("Failed to save UI screenshot: " + screenshot_path);
            } else {
                appendStatus("UI screenshot saved: " + screenshot_path);
            }
            QTimer::singleShot(50, qApp, [ok]() { QCoreApplication::exit(ok ? 0 : 1); });
        });
    }

    QComboBox* dataset_combo_ = nullptr;
    QComboBox* buffer_dataset_combo_ = nullptr;
    QLineEdit* land_path_edit_ = nullptr;
    QLineEdit* tss_path_edit_ = nullptr;
    QLineEdit* bathymetry_path_edit_ = nullptr;
    QLineEdit* buffer_dataset_path_edit_ = nullptr;
    QLineEdit* svg_output_edit_ = nullptr;
    QLineEdit* navigation_svg_edit_ = nullptr;
    QLineEdit* triangulation_cache_edit_ = nullptr;
    QPushButton* land_browse_button_ = nullptr;
    QPushButton* tss_browse_button_ = nullptr;
    QPushButton* bathymetry_browse_button_ = nullptr;
    QPushButton* buffer_dataset_browse_button_ = nullptr;
    QPushButton* svg_browse_button_ = nullptr;
    QPushButton* navigation_svg_browse_button_ = nullptr;
    QPushButton* triangulation_cache_browse_button_ = nullptr;
    QPushButton* render_button_ = nullptr;
    QPushButton* fit_button_ = nullptr;
    QPushButton* load_navigation_svg_button_ = nullptr;
    QPushButton* load_triangulation_layer_button_ = nullptr;
    QPushButton* pick_start_button_ = nullptr;
    QPushButton* pick_goal_button_ = nullptr;
    QPushButton* cancel_pick_button_ = nullptr;
    QPushButton* plan_route_button_ = nullptr;
    QCheckBox* route_tss_checkbox_ = nullptr;
    QCheckBox* tss_enabled_checkbox_ = nullptr;
    QCheckBox* bathymetry_enabled_checkbox_ = nullptr;
    QCheckBox* buffer_zones_enabled_checkbox_ = nullptr;
    QCheckBox* custom_bounds_checkbox_ = nullptr;
    QDoubleSpinBox* min_lat_spin_ = nullptr;
    QDoubleSpinBox* max_lat_spin_ = nullptr;
    QDoubleSpinBox* min_lon_spin_ = nullptr;
    QDoubleSpinBox* max_lon_spin_ = nullptr;
    QDoubleSpinBox* route_start_lat_spin_ = nullptr;
    QDoubleSpinBox* route_start_lon_spin_ = nullptr;
    QDoubleSpinBox* route_goal_lat_spin_ = nullptr;
    QDoubleSpinBox* route_goal_lon_spin_ = nullptr;
    QPlainTextEdit* status_box_ = nullptr;
    QLabel* loading_label_ = nullptr;
    QProgressBar* loading_progress_ = nullptr;
    QLabel* pick_status_label_ = nullptr;
    ZoomGraphicsView* preview_view_ = nullptr;
    QGraphicsScene* scene_ = nullptr;
    QGraphicsPathItem* triangulation_overlay_item_ = nullptr;
    QGraphicsPathItem* route_overlay_item_ = nullptr;
    QGraphicsPathItem* start_marker_item_ = nullptr;
    QGraphicsPathItem* goal_marker_item_ = nullptr;
    QFutureWatcher<RenderTaskResult> render_watcher_;
    QFutureWatcher<TriangulationLayerTaskResult> triangulation_watcher_;
    QFutureWatcher<RoutePlanningTaskResult> route_watcher_;
    std::vector<QRectF> zoom_history_;
    QString current_svg_path_;
    QString route_cache_path_;
    QString current_render_land_dataset_;
    QString current_render_bathymetry_dataset_;
    QString startup_screenshot_path_;
    bool startup_screenshot_done_ = false;
    PickMode pick_mode_ = PickMode::None;
    RenderTaskResult current_geo_transform_;
};

}  // namespace

int main(int argc, char** argv) {
    QApplication app(argc, argv);

    QString startup_screenshot_path;
    for (int i = 1; i < argc; ++i) {
        const QString arg = QString::fromLocal8Bit(argv[i]);
        if (arg == "--screenshot") {
            if (i + 1 >= argc) {
                qCritical("Missing value for --screenshot");
                return 1;
            }
            startup_screenshot_path = QString::fromLocal8Bit(argv[++i]);
            continue;
        }
        qWarning("Unknown argument: %s", argv[i]);
    }

    ViewerWindow window(startup_screenshot_path);
    window.show();
    return app.exec();
}
