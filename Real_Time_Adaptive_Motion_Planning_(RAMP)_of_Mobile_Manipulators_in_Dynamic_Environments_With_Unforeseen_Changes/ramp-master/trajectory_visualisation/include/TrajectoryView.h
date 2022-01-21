/****************************************************************************
**
** Copyright (C) 2010 Nokia Corporation and/or its subsidiary(-ies).
** All rights reserved.
** Contact: Nokia Corporation (qt-info@nokia.com)
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of Nokia Corporation and its Subsidiary(-ies) nor
**     the names of its contributors may be used to endorse or promote
**     products derived from this software without specific prior written
**     permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef TRAJECTORYVIEW_H
#define TRAJECTORYVIEW_H

#include <QtGui/QGraphicsView>
#include "ramp_msgs/Population.h"
#include "utility.h"
#include <vector>
#include <ros/console.h>


class TrajectoryView : public QGraphicsView
 {
     Q_OBJECT

 public:
     TrajectoryView(QWidget *parent = 0);
     void size_changed(void); // Change the scene size to the updated one when the user resizes the window
     void size_changed_manual();

 public slots:
     void population(const ramp_msgs::Population& msg); //receive the list of trajectories and display them

 signals:

 protected:


 private:
     int height_; // the height of the scene
     int width_; // the width of the scene

     const std::vector<float> getCenter(std::vector<float> p, float orientation) const;
     
     Utility u;
     std::vector<ramp_msgs::Population> populations_; // the list of trajectories. The first one has to be the best and is displayed in red.
     
     float maxWidthMeters_; // The maximum x value amoung all the points in the trajectories
     float maxHeightMeters_; // The maximum y value amoung all the points in the trajectories

     void drawPopulation(); // Draw the trajectories on the scene
     int const metersToPixels(float value, bool width); //Calculate the pixel value of a distance. If width is true, treat the value has a x position, if false treat it as a y position.


 };



#endif // TRAJECTORYVIEW_H

