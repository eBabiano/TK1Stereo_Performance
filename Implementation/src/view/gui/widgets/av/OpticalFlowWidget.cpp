#include <src/view/gui/widgets/av/OpticalFlowWidget.hpp>
#include "ui_OpticalFlowWidget.h"

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
                    OpticalFlowWidget::OpticalFlowWidget(model::av::AVManager &avManager, QWidget *parent) :
                        AVWidget(avManager, parent) ,
                        ui(new Ui::OpticalFlowWidget)
                    {
                        ui->setupUi(this);
                    }

                    OpticalFlowWidget::~OpticalFlowWidget()
                    {
                        delete ui;
                    }

                }
            }
        }
    }
}
