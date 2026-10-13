// Layer Slicer viewer.
//
//   layer_slicer_viewer [model.stl] [--layer N] [--layer-height H] [--spacing S]
//                       [--screenshot out.png] [--size WxH]
//
// With --screenshot the window is rendered offscreen (never shown on screen)
// into a PNG and the program exits. For machines without a display use
// QT_QPA_PLATFORM=offscreen.
#include <QApplication>
#include <QCommandLineParser>
#include <QImage>
#include <QPixmap>
#include <QTextStream>
#include <cstdio>

#include "main_window.h"
#include "slice_view.h"

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    QApplication::setApplicationName(QStringLiteral("Layer Slicer"));

    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral("Slice STL meshes into layers and show contours and scan paths."));
    parser.addHelpOption();
    parser.addPositionalArgument(QStringLiteral("model"), QStringLiteral("STL file to open."));
    const QCommandLineOption layerOpt(QStringLiteral("layer"), QStringLiteral("Layer index to show."),
                                      QStringLiteral("n"));
    const QCommandLineOption heightOpt(QStringLiteral("layer-height"), QStringLiteral("Layer thickness."),
                                       QStringLiteral("h"));
    const QCommandLineOption spacingOpt(QStringLiteral("spacing"), QStringLiteral("Infill hatch spacing."),
                                        QStringLiteral("s"));
    const QCommandLineOption shotOpt(QStringLiteral("screenshot"),
                                     QStringLiteral("Render the window to a PNG file and exit."),
                                     QStringLiteral("png"));
    const QCommandLineOption sizeOpt(QStringLiteral("size"), QStringLiteral("Window size, e.g. 1200x820."),
                                     QStringLiteral("WxH"));
    parser.addOptions({layerOpt, heightOpt, spacingOpt, shotOpt, sizeOpt});
    parser.process(app);

    MainWindow window;
    if (parser.isSet(sizeOpt)) {
        const QStringList wh = parser.value(sizeOpt).split(QLatin1Char('x'));
        if (wh.size() == 2) window.resize(wh[0].toInt(), wh[1].toInt());
    }
    if (parser.isSet(heightOpt)) window.setLayerHeight(parser.value(heightOpt).toDouble());
    if (parser.isSet(spacingOpt)) window.setHatchSpacing(parser.value(spacingOpt).toDouble());

    const QStringList args = parser.positionalArguments();
    if (!args.isEmpty()) {
        QString error;
        if (!window.loadFile(args.first(), &error)) {
            std::fprintf(stderr, "%s\n", qPrintable(error));
            return 1;
        }
    }
    if (parser.isSet(layerOpt)) {
        const int n = parser.value(layerOpt).toInt();
        if (n < 0 || n >= window.layerCount()) {
            std::fprintf(stderr, "layer %d out of range (model has %d layers)\n", n, window.layerCount());
            return 1;
        }
        window.setLayer(n);
    }

    if (parser.isSet(shotOpt)) {
        // Render without ever putting the window on screen. Works with the
        // native platform plugin and with QT_QPA_PLATFORM=offscreen.
        window.setAttribute(Qt::WA_DontShowOnScreen, true);
    }
    window.show();
    if (parser.isSet(shotOpt)) {
        QApplication::processEvents();
        window.view()->fitToView();
        QApplication::processEvents();
        const QString out = parser.value(shotOpt);
        if (!window.grab().save(out, "PNG")) {
            std::fprintf(stderr, "could not write %s\n", qPrintable(out));
            return 1;
        }
        std::printf("wrote %s\n", qPrintable(out));
        return 0;
    }
    return QApplication::exec();
}
