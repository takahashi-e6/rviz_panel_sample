/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

// ros headers here
#include <ros/ros.h>
#include <rviz/panel.h>
#include <visualization_msgs/Marker.h>


// qt headers here
#include <QtCore>
#include <QPushButton>

namespace rviz_plugin
{
// rviz::Panelを継承しているのが、ポイント
class SamplePanel : public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  SamplePanel( QWidget* parent = 0 );
  ~SamplePanel();

private:
  bool tgl_;                  // ボタンの押下状態を保持する変数
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_; // マーカメッセージパブリッシャ
  QPushButton* btn_;
  double x_, y_;
  visualization_msgs::Marker create_marker( const double& x, const double& y ); // スロットはシグナル（イベント）が発生した時に呼ばれる
protected Q_SLOTS:
  void btn_clicked();     // ボタンが押された時のイベント
  void sld_x_changed(int);  // スライダーの値が変わった時のイベント
  void sld_y_changed(int);  // スライダーの値が変わった時のイベント
};

} // end namespace rviz_plugin_tutorials
