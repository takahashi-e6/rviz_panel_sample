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
#include "sample.h"

#include <QString>
#include <QSlider>
#include <QHBoxLayout>

using namespace std;

namespace rviz_plugin
{
SamplePanel::SamplePanel( QWidget* parent )
  : rviz::Panel( parent ),
  tgl_(true),
  x_(0.0),
  y_(0.0)
{
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/sample_marker", 10);

  // パネルGUIのレイアウト作成
  QHBoxLayout* layout = new QHBoxLayout;  // 水平配置する
  btn_ = new QPushButton( "Spawn" );      // ボタン作成
  QSlider* sld_x =  new QSlider(Qt::Horizontal);
  QSlider* sld_y =  new QSlider(Qt::Horizontal);
  sld_x->setRange(1, 100);                   // スライダーの範囲を決める
  sld_y->setRange(1, 100);                   // スライダーの範囲を決める
  layout->addWidget( btn_ );              // ボタンをレイアウトへ登録
  layout->addWidget( sld_x );               // スライダーをレイアウトへ登録
  layout->addWidget( sld_y );               // スライダーをレイアウトへ登録

  this->setLayout(layout);                // RvizパネルへGUIをのせる

  // イベントSIGNALとイベント時のコールバックSLOTを設定
  connect( btn_, SIGNAL( clicked() ), this, SLOT( btn_clicked() ));
  connect( sld_x, SIGNAL( valueChanged(int) ), this, SLOT( sld_x_changed(int)) );
  connect( sld_y, SIGNAL( valueChanged(int) ), this, SLOT( sld_y_changed(int)) );
}


SamplePanel::~SamplePanel(){}

void SamplePanel::btn_clicked()
{
  visualization_msgs::Marker marker;

  // マーカ表示
  if( tgl_ )
  {
    btn_->setText("Delete");
    marker = create_marker( x_, y_ );
  }
  // マーカ削除
  else
  {
    btn_->setText("Spawn");
    marker.action = visualization_msgs::Marker::DELETE;
    marker.id = 0;
  }
  tgl_ = !tgl_;
  marker_pub_.publish(marker);
}


void SamplePanel::sld_x_changed( int val )
{
  x_ = (double)val / 10.0;
  visualization_msgs::Marker marker = create_marker( x_, y_ );
  marker_pub_.publish(marker);
}


void SamplePanel::sld_y_changed( int val )
{
  y_ = (double)val / 10.0;
  visualization_msgs::Marker marker = create_marker( x_, y_ );
  marker_pub_.publish(marker);
}


visualization_msgs::Marker SamplePanel::create_marker( const double& x, const double& y )
{
  visualization_msgs::Marker marker;
  std_msgs::ColorRGBA color;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.5;
  color.a = 1.0;
  color.r = 1.0;
  marker.color = color;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.orientation.w = 1;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  return marker;
}

}// end of namespace rviz_plugin

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin::SamplePanel, rviz::Panel )
