#include <stairs/pclviewer.h>
#include <QApplication>
#include <QMainWindow>
#include <QFileDialog>

int main (int argc, char *argv[])
{  
  QApplication a (argc, argv);
  PCLViewer w;
  w.show ();

  return a.exec ();
}
