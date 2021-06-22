/*
Copyright (c) 2019 TOYOTA MOTOR CORPORATION
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#include <cmath>
#include <sstream>
#include "score_widget.hh"

using namespace gazebo;

GZ_REGISTER_GUI_PLUGIN(ScoreWidget)

ScoreWidget::ScoreWidget() : GUIPlugin()
{
  this->setStyleSheet(
      "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  QHBoxLayout *mainLayout = new QHBoxLayout;
  QFrame *mainFrame = new QFrame();
  QHBoxLayout *frameLayout = new QHBoxLayout();
  QLabel *label = new QLabel(tr("Score:"));
  QLabel *scoreLabel = new QLabel(tr("   0"));

  // Add the label to the frame's layout
  frameLayout->addWidget(label);
  frameLayout->addWidget(scoreLabel);
  connect(this, SIGNAL(SetScore(QString)),
      scoreLabel, SLOT(setText(QString)), Qt::QueuedConnection);

  mainFrame->setLayout(frameLayout);

  mainLayout->addWidget(mainFrame);

  frameLayout->setContentsMargins(4, 4, 4, 4);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  this->move(10, 10);
  this->resize(120, 30);

  // initialize ROS node
  int argc = 0;
  ros::init(argc, NULL, "score_widget");
  ros::NodeHandle n;
  sub = n.subscribe("score", 1, &ScoreWidget::OnScore, this);
  spinner.reset(new ros::AsyncSpinner(1));
  spinner->start();
  ros::AsyncSpinner spinner(1);
  spinner.start();
}

ScoreWidget::~ScoreWidget()
{
}

void ScoreWidget::OnScore(const std_msgs::Float32Ptr& score)
{
  this->SetScore(QString::fromStdString(this->FormatScore(score->data)));
}

std::string ScoreWidget::FormatScore(double score)
{
  std::ostringstream stream;
  stream.str("");
  stream << std::setw(3) << std::setfill(' ') << std::round(score);
  return stream.str();
}
