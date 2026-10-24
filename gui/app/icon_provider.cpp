// gui/app/icon_provider.cpp

#include "icon_provider.h"

#include <QFile>
#include <QGuiApplication>
#include <QHash>
#include <QPalette>
#include <QPainter>
#include <QPixmap>
#include <QSvgRenderer>

namespace gui {

namespace {
// Cache of rendered icons keyed by "name|color". Avoids repeated file I/O
// and SVG parsing on theme refreshes (§14.2 — performance fix).
// The cache grows with the number of distinct (name, color) pairs, which is
// bounded by (icon count × distinct colors) — typically < 100 entries.
QHash<QString, QIcon>& icon_cache() {
    static QHash<QString, QIcon> cache;
    return cache;
}

QString cache_key(const QString& name, const QColor& color) {
    return name + QLatin1Char('|') + color.name();
}
} // namespace

QIcon IconProvider::get(const QString& name) {
    // The ThemeController sets the application palette's WindowText/Text to the
    // current theme's primary foreground color, so reading the palette here
    // keeps icons in sync with the active theme without a direct dependency.
    QColor fg = qApp->palette().color(QPalette::WindowText);
    if (!fg.isValid()) fg = QColor(Qt::black);
    return render(name, fg);
}

QIcon IconProvider::get(const QString& name, const QColor& color) {
    return render(name, color.isValid() ? color : QColor(Qt::black));
}

void IconProvider::clear_cache() {
    icon_cache().clear();
}

QIcon IconProvider::render(const QString& name, const QColor& color) {
    // Check the cache first — avoids file I/O + SVG parsing on repeated
    // requests for the same icon (e.g. during theme refresh).
    const QString key = cache_key(name, color);
    auto& cache = icon_cache();
    auto it = cache.constFind(key);
    if (it != cache.constEnd()) return it.value();

    QFile f(QStringLiteral(":/icons/") + name + QStringLiteral(".svg"));
    if (!f.open(QIODevice::ReadOnly)) return QIcon();

    QString svg = QString::fromUtf8(f.readAll());
    // Recolor: every "currentColor" occurrence becomes the target color so
    // both stroke-based and fill-based icons pick up the requested color.
    svg.replace(QStringLiteral("currentColor"), color.name());

    // Render via QSvgRenderer (Qt6::Svg, linked at build time). Going through
    // the plugin-based QImageReader instead would depend on the qt6-svg
    // imageformat plugin being present at runtime, which is not guaranteed in
    // packaged bundles (macOS .app, Windows installs).
    QSvgRenderer renderer(svg.toUtf8());
    if (!renderer.isValid()) return QIcon();

    const QSize size = renderer.defaultSize().isEmpty()
                           ? QSize(24, 24)
                           : renderer.defaultSize();
    QPixmap pm(size);
    pm.fill(Qt::transparent);
    QPainter painter(&pm);
    renderer.render(&painter);
    if (pm.isNull()) return QIcon();
    QIcon icon(pm);
    cache.insert(key, icon);
    return icon;
}

} // namespace gui
