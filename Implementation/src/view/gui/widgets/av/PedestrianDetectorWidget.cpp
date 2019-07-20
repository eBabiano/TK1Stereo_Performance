#include <src/view/gui/widgets/av/PedestrianDetectorWidget.hpp>
#include "ui_PedestrianDetectorWidget.h"
#include <src/model/av/AVTypes.hpp>

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
                    PedestrianDetectorWidget::PedestrianDetectorWidget(model::av::AVManager &avManager, QWidget *parent) :
                        AVWidget(avManager, parent) ,
                        ui(new Ui::PedestrianDetectorWidget)
                    {
                        ui->setupUi(this);

                        if (mAVManager->getAlgorithm(model::av::AVTypes::PEDESTRIAN_DETECTOR)->
                                getProcessorType() == model::av::AVTypes::CPU)
                        {
                            ui->cpuRadioButton->setChecked(true);
                        }
                        else
                        {
                            ui->gpuRadioButton->setChecked(true);
                        }
                    }

                    PedestrianDetectorWidget::~PedestrianDetectorWidget()
                    {
                        delete ui;
                    }

                    void PedestrianDetectorWidget::on_cpuRadioButton_toggled(bool checked)
                    {
                        ui->gpuRadioButton->setChecked(false);
                        notify(events::ModifyPedestrianDetectorEvent(src::model::av::AVTypes::CPU));
                    }

                    void PedestrianDetectorWidget::on_gpuRadioButton_toggled(bool checked)
                    {
                        ui->cpuRadioButton->setChecked(false);
                        notify(events::ModifyPedestrianDetectorEvent(src::model::av::AVTypes::GPU));
                    }

                }
            }
        }
    }
}
