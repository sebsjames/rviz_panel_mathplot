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

#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <rviz_panel_mathplot/mathplot_panel.hpp>
#include <sm/vec>
#include <sm/vvec>
#include <mplot/qt/viswidget_mx.h>
#include <mplot/GraphVisual.h>

namespace rviz_panel_mathplot
{
    static constexpr int VWID = 0;

    MathplotPanel::MathplotPanel (QWidget * parent) : Panel(parent)
    {
        // Create a label and a button, displayed vertically (the V in VBox means vertical)
        auto layout = new QVBoxLayout(this);
        label_ = new QLabel("[no data]");
        button_ = new QPushButton("GO!");
        //layout->addWidget(label_);
        this->viswidget_init (layout);
        layout->addWidget(button_);

        // Connect the event of when the button is released to our callback,
        // so pressing the button results in the callback being called.
        QObject::connect(button_, &QPushButton::released, this, &MathplotPanel::buttonActivated);
    }

    MathplotPanel::~MathplotPanel() = default;

    void MathplotPanel::onInitialize()
    {
        // Access the abstract ROS Node and
        // in the process lock it for exclusive use until the method is done.
        node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

        // rviz_common::Panel provides getDisplayContext which returns the VisualizationManager!
        //getDisplayContext()->lockRender();

        // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
        // (as per normal rclcpp code)
        rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
        publisher_ = node->create_publisher<std_msgs::msg::String>("/output", 10);
        subscription_ = node->create_subscription<std_msgs::msg::String>(
            "/input", 10, std::bind(&MathplotPanel::topicCallback, this, std::placeholders::_1));
    }

    // When the subscriber gets a message, this callback is triggered,
    // and then we copy its data into the widget's label
    void MathplotPanel::topicCallback (const std_msgs::msg::String& msg)
    {
        label_->setText(QString(msg.data.c_str()));
    }

    // When the widget's button is pressed, this callback is triggered,
    // and then we publish a new message on our topic.
    void MathplotPanel::buttonActivated()
    {
        auto message = std_msgs::msg::String();
        message.data = "Button clicked!";
        publisher_->publish(message);

        std::cout << "Adding a GraphVisual...\n";

        auto gv = std::make_unique<mplot::GraphVisual<double, mplot::qt::gl_version>> (this->graphlocn);
        // Bind the new (Graph)VisualModel to the mplot::Visual associated with the viswidget
        static_cast<mplot::qt::viswidget_mx<VWID>*>(this->p_vw)->v.bindmodel (gv);

        gv->twodimensional = false;
        sm::vvec<double> x;
        x.linspace (-1.5, 1.5, 25);
        gv->setdata (x, x.pow(2));

        // Cast and add
        std::unique_ptr<mplot::VisualModel<mplot::qt::gl_version>> vmp = std::move (gv);
        static_cast<mplot::qt::viswidget_mx<VWID>*>(this->p_vw)->newvisualmodels.push_back (std::move(vmp));

        // request a render, otherwise it won't appear until user interacts with window
        this->p_vw->update();

        // Change the graphlocn so that the next graph shows up in a different place
        this->graphlocn[1] += 1.2f;
    }

    //void MathplotPanel::get_threadlock (void* p) { reinterpret_cast<MathplotPanel*>(p)->getDisplayContext()->lockRender(); }
    //void MathplotPanel::release_threadlock (void* p) { reinterpret_cast<MathplotPanel*>(p)->getDisplayContext()->unlockRender(); }

    void MathplotPanel::viswidget_init (QLayout* owner_layout)
    {
        // Create widget. Seems to open in its own window with a new context.
        rviz_panel_mathplot::panel_viswidget<VWID>* vw = new rviz_panel_mathplot::panel_viswidget<VWID> (this->parentWidget());
        // Choose lighting effects if you want
        vw->v.lightingEffects();

        // Set a callback in the viswidget so that it can call lockRender and unlockRender
        //vw->threadlock_parent = reinterpret_cast<void*>(this);
        //vw->get_threadlock = &rviz_panel_mathplot::MathplotPanel::get_threadlock;
        //vw->release_threadlock = &rviz_panel_mathplot::MathplotPanel::release_threadlock;
        // Perhaps I need to extend viswidget_mx so that it knows what a MathplotPanel
        // is? Then it could have access to getDisplayContext without callbacks.

        // Add the OpenGL widget to the UI.
        owner_layout->addWidget (vw);
        // Keep a copy of vw
        this->p_vw = vw;
    }

    template<int widget_index>
    panel_viswidget<widget_index>::panel_viswidget (QWidget* parent) : mplot::qt::viswidget_mx<widget_index>(parent) {}

#if 0
    template<int widget_index>
    panel_viswidget<widget_index>::set_panelparent (rviz_panel_mathplot::MathplotPanel *pp)
    {
        panelparent = pp;
    }
#endif
#if 1
    template<int widget_index>
    panel_viswidget<widget_index>::getlock()
    { if (this->panelparent) { this->panelparent->getDisplayContext()->lockRender(); } }

    template<int widget_index>
    panel_viswidget<widget_index>::releaselock()
    { if (this->panelparent) { this->panelparent->getDisplayContext()->unlockRender(); } }
#endif

}  // namespace rviz_panel_mathplot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_panel_mathplot::MathplotPanel, rviz_common::Panel)
