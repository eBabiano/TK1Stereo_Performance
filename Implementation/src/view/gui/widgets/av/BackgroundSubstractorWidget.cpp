#include <src/view/gui/widgets/av/BackgroundSubstractorWidget.hpp>
#include "ui_BackgroundSubstractorWidget.h"

namespace src
{
    namespace view
    {
        namespace gui
        {
            namespace widgets
            {
                namespace av
                {
                    BackgroundSubstractorWidget::BackgroundSubstractorWidget(model::av::AVManager& avManager, QWidget *parent) :
                        AVWidget(avManager, parent),
                        ui(new Ui::BackgroundSubstractorWidget)
                    {
                        ui->setupUi(this);
                    }

                    BackgroundSubstractorWidget::~BackgroundSubstractorWidget()
                    {
                        delete ui;
                    }

                    void BackgroundSubstractorWidget::on_horizontalSlider_valueChanged(int value)
                    {
                        notify(events::ModifyBackgroundSubstractorEvent(value));
                    }

                }
            }
        }
    }
}
