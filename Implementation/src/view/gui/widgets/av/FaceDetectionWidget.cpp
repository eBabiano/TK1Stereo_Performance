#include <src/view/gui/widgets/av/FaceDetectionWidget.hpp>
#include "ui_FaceDetectionWidget.h"
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
                    FaceDetectionWidget::FaceDetectionWidget(model::av::AVManager &avManager, QWidget *parent) :
                        AVWidget(avManager, parent)
                        , ui(new Ui::FaceDetectionWidget)
                    {
                        ui->setupUi(this);

                        if (mAVManager->getAlgorithm(model::av::AVTypes::FACE_DETECTION)->
                                getProcessorType() == model::av::AVTypes::CPU)
                        {
                            ui->cpuRadioButton->setChecked(true);
                        }
                        else
                        {
                            ui->gpuRadioButton->setChecked(true);
                        }
                    }

                    FaceDetectionWidget::~FaceDetectionWidget()
                    {
                        delete ui;
                    }

                    void FaceDetectionWidget::on_cpuRadioButton_toggled(bool checked)
                    {
                        ui->gpuRadioButton->setChecked(false);
                        notify(events::ModifyFaceDetectionEvent(src::model::av::AVTypes::CPU));
                    }

                    void FaceDetectionWidget::on_gpuRadioButton_toggled(bool checked)
                    {
                        ui->cpuRadioButton->setChecked(false);
                        notify(events::ModifyFaceDetectionEvent(src::model::av::AVTypes::GPU));
                    }

                }
            }
        }
    }
}
