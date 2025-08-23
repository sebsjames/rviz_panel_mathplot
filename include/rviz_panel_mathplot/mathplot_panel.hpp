/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Metro Robots
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Metro Robots nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: David V. Lu!!
   Author: Seb James
 */

#ifndef RVIZ_PANEL_MATHPLOT__MATHPLOT_PANEL_HPP_
#define RVIZ_PANEL_MATHPLOT__MATHPLOT_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>
#include <sm/vec>

class QOpenGLWidget;
class QLayout;

namespace rviz_panel_mathplot
{
    class MathplotPanel : public rviz_common::Panel // Panel derives from QWidget
    {
        Q_OBJECT
    public:
        explicit MathplotPanel (QWidget* parent = 0);
        ~MathplotPanel() override;

        void onInitialize() override;

    protected:
        std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

        // Would like VisualizationManager pointer here too

        void topicCallback (const std_msgs::msg::String& msg);

        // Initialise your viswidget. In this example, one widget is created and
        // added to a layout.
        void viswidget_init (QLayout* owner_layout);

        // A pointer (in Qt-speak) to your viswidget, which is a part of your overall Qt Window
        QOpenGLWidget* p_vw = nullptr;
        QLabel* label_;
        QPushButton* button_;

        // A location for a graph within the Visual scene inside the viswidget
        sm::vec<float, 3> graphlocn = {1.5f, 0.0f, 0.0f};

    private Q_SLOTS:
        void buttonActivated();
        // These are signals that the viswidget may emit
        void aboutToCompose();
        void aboutToResize();
        void frameSwapped();
        void resized();
    };

}  // namespace rviz_panel_mathplot

#endif  // RVIZ_PANEL_MATHPLOT__MATHPLOT_PANEL_HPP_
