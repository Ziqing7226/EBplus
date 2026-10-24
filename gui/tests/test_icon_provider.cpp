// gui/tests/test_icon_provider.cpp -- embedded SVG rendering regression test.

#include <gtest/gtest.h>

#include <QApplication>
#include <QIcon>
#include <QPixmap>

#include "app/icon_provider.h"

int main(int argc, char** argv) {
    qputenv("QT_QPA_PLATFORM", "offscreen");
    QApplication app(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(IconProvider, RendersEmbeddedSvgToPixmap) {
    const QIcon icon = gui::IconProvider::get(QStringLiteral("camera"));
    EXPECT_FALSE(icon.isNull());

    const QPixmap pixmap = icon.pixmap(QSize(20, 20));
    EXPECT_FALSE(pixmap.isNull());
    EXPECT_EQ(pixmap.size(), QSize(20, 20));
}
