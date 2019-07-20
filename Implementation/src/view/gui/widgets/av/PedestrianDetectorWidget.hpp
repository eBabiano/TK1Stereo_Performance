#ifndef PEDESTRIANDETECTORWIDGET_HPP
#define PEDESTRIANDETECTORWIDGET_HPP

#include <QWidget>
#include <src/view/gui/widgets/av/AVWidget.hpp>

namespace Ui {
class PedestrianDetectorWidget;
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
                    class PedestrianDetectorWidget
                            : public AVWidget
                    {
                        Q_OBJECT
                    public:
                        explicit PedestrianDetectorWidget(model::av::AVManager& avManager, QWidget *parent = 0);
                        ~PedestrianDetectorWidget();

                    private slots:
                        void on_cpuRadioButton_toggled(bool checked);
                        void on_gpuRadioButton_toggled(bool checked);

                    private:
                        Ui::PedestrianDetectorWidget *ui;

                    };
                }
            }
        }
    }
}

#endif // PEDESTRIANDETECTORWIDGET_HPP
