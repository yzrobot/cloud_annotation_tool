#include <QApplication>
#include <QMainWindow>
#include "viewer.h"

int main(int argc, char **argv) {
  QApplication a(argc, argv);
  CloudViewer w;
  w.show();
  return a.exec();
}
