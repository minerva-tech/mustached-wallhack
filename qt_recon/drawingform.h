#ifndef DRAWINGFORM_H
#define DRAWINGFORM_H

#include <QWidget>

namespace Ui {
class DrawingForm;
}

class DrawingForm : public QWidget
{
    Q_OBJECT

public:
    explicit DrawingForm(QWidget *parent = 0);
    ~DrawingForm();

private:
    Ui::DrawingForm *ui;
};

#endif // DRAWINGFORM_H
