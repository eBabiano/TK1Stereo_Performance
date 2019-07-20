#ifndef OPTICALFLOWWIDGET_HPP
#define OPTICALFLOWWIDGET_HPP

#include <QWidget>
#include <src/view/gui/widgets/av/AVWidget.hpp>
#include <src/view/gui/events/ModifyOpticalFlowEvent.hpp>

namespace Ui {
class OpticalFlowWidget;
}

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
                    class OpticalFlowWidget
                           : public AVWidget
                    {
                        Q_OBJECT

                    public:
                        explicit OpticalFlowWidget(model::av::AVManager &avManager, QWidget *parent = 0);
                        ~OpticalFlowWidget();


                    private:
                        Ui::OpticalFlowWidget *ui;

                    };
                }
            }
        }
    }
}

#endif // OPTICALFLOWWIDGET_HPP
