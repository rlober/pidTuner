/*!
*  \file       main.cpp
*  \brief      Creates and launches the pidTunerGui module.
*  \author     Ryan Lober
*  \email      ryan.lober@isir.upmc.com
*  \version    1.0
*  \date       28 May 2015
*  \copyright  GNU Public License.
*
* This file is part of pidTuner, a set of modules for manual
* tuning of individual joint low level gains on the iCub robot.
* For more details please look at the README or check out the
* Github repository: https://github.com/rlober/pidTuner.git
*
* pidTuner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 2 of the License, or
* (at your option) any later version.
*
* pidTuner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with pidTuner.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "mainwindow.h"
#include <QApplication>
#include <yarp/os/ResourceFinder.h>

int main(int argc, char *argv[])
{

    yarp::os::ResourceFinder rf;
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        std::cout<< "Possible parameters" << "\n\n";
        std::cout<< "\t--exclude :A part you wish to exclude. Set to empty by default." <<std::endl;
        return 0;
    }

    QApplication a(argc, argv);

    MainWindow w(rf);
    w.setWindowTitle("PID Tuner");
    w.show();



    return a.exec();
}
