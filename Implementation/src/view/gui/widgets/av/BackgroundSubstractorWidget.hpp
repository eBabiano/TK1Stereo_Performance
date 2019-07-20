#ifndef BackgroundSubstractorWidget_HPP
#define BackgroundSubstractorWidget_HPP

#include <QWidget>
#include <src/view/gui/widgets/av/AVWidget.hpp>
#include <stdio.h>

namespace Ui {
class BackgroundSubstractorWidget;
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
                    class BackgroundSubstractorWidget
                            : public AVWidget
                    {
                        Q_OBJECT

                    public:
                        explicit BackgroundSubstractorWidget(model::av::AVManager &avManager, QWidget *parent = 0);
                        ~BackgroundSubstractorWidget();

                    private slots:
                        void on_horizontalSlider_valueChanged(int value);

                    private:
                        Ui::BackgroundSubstractorWidget *ui;


                    };
                }
            }
        }
    }
}

#endif // BackgroundSubstractorWidget_HPP
