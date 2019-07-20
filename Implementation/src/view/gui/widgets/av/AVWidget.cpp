#include <src/view/gui/widgets/av/AVWidget.hpp>

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
                    AVWidget::AVWidget(model::av::AVManager& avManager, QWidget *parent)
                        : QWidget(parent)
                        , mAVManager(&avManager)
                    {
                    }

                    void AVWidget::hidden()
                    {
                        setVisible(false);
                    }
                }
            }
        }
    }
}
