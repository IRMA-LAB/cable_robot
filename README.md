# GRAB Cable Robot <img align="right" src="https://www.gnu.org/graphics/gplv3-127x51.png">

## Overview

This repository contains the code to control and operate a custom cable-driven robot built at GRAB laboratory. 

It relies on [_EtherCAT_](https://www.ethercat.org/default.htm) network system and requires a _Real Time Linux Operating System_. Both robot controller and human interface are handled by the same working station, i.e. a standard PC with RT characteristic, and covered here.

## Installation

Follow [these](https://github.com/UNIBO-GRABLab/cable_robot/wiki/Installation) instructions thoroughly to get you a copy of the project up and running on your local machine for development and testing purposes. Please contact a maintaner if you incur any trouble.

To clone this repository, open a terminal, cd to where you want this local copy to be downloaded and type the following commands:
```bash
git clone git@github.com:UNIBO-GRABLab/cable_robot.git
git submodule update --init
```
The second command is absolutely necessary to download the latest versions of the submodules on which this repository relies on.

## Build

This application depends on [grabcommon libraries](https://github.com/UNIBO-GRABLab/grab_common). Keeping this in mind, the user is free to build it as desired.

We provide here a Qt project file and instructions for compiling this package using _QtCreator_. Please note that this builing process **requires** a compiled version of both _state_machine_ and _grabcommon_ static libraries.

### Prerequisites

If not yet present on your machine, please install _QtCreator_ together with _Qt_. To do so, open a terminal a type the following commands:

```bash
sudo apt-get install build-essential libgl-dev libicu-dev
sudo apt-get install qtcreator
sudo apt-get install qt5-default
```
Because this application relies on etherlab libraries and real-time scheduling, it needs admin access to your system at run time. To do so you can simply start your application from terminal with administrator privliges, such as:
```bash
sudo qtcreator
```
To avoid to open a new terminal every time you want to work on this project, we suggest to use a shortcut using _gksudo_:
1. Install _gksudo_: 
```bash
sudo apt-get install gksu
```
2. Open Qt launcher with your preferred editor, such as: 
```bash
sudo gedit /usr/share/applications/qt59creator.desktop
```
Be aware that your Qt .desktop file might have slightly different name.
3. In the _Exec_ field, prepend the following command to what is already written: `gksudo -k -u root `.
For instance, `Exec=opt/qt59/bin/qtcreator %F` becomes `Exec=gksudo -k -u root opt/qt59/bin/qtcreator %F`.

Now whenever you will click on Qt icon you will be prompted to a window asking for administrator password.

### Import and run CableRobotApp Qt project

1. Open QtCreator.
2. In the **Welcome** side tab, in **Projects** group, select **Open Project**, browser to your local copy this repository and open _cable_robot.pro_.
3. Click the hammer button on the bottom left to build it and the green play button right above to run the application.

## Usage

Please refer to [this wiki section](https://github.com/UNIBO-GRABLab/cable_robot/wiki/Usage) for more details about how to use this application.

## Documentation

To auto-generate an user-friendly documentation straight out of the source code, we make use of well-known software tool _[Doxygen](http://www.stack.nl/~dimitri/doxygen/)_. On the official website you can find all the details about how to use it, install it and build it. While we provide here a short tutorial on how to integrate it in our Qt-based framework, we leave to the user the task of reading the [manual](http://www.stack.nl/~dimitri/doxygen/manual/docblocks.html) on the format to be employed when writing documentation within the code. Bear in mind that in this project we opted for _JavaDoc_ style, so please comply with it.

### Prerequisites

First of all, you need to download the tool itself (you can skip this step if it is already installed in your machine).
To do so, open a terminal and type the following commands:
```bash
sudo apt-get install doxygen doxygen-gui
```
That's it. Now Doxygen is installed and ready to be used.

### Setup a Doxygen project with _doxywizard_

Next, you need to setup a _Doxyfile_, which yields all the instructions to generate the documentation out of your code. Instead of compiling this file yourself, _Doxygen_ provides a nice wizard to walk you throug all the necessary step: _doxywizard_.
To start it, just type `doxywizard` in a terminal and a small GUI will appear.
Feel free to adapt the settings to your specific need, but the minimal setup for this library should include the following:
- In `Step 1`, select _$MYPROJECTPATH_, where _$MYPROJECTPATH_ is the absolute path to your project root directory.
- In `Step 2`, in the tab `Wizard`, select `Project` under `Topics` and do the following:
  - Fill project information, such as `Project name`, as desired
  - Select _$MYPROJECTPATH_ as `Source code directory` and tick `Scan recursively` option below
  - Select _$MYPROJECTPATH/doc_ as `Destination directory` (make directory if it does not exist yet)
- In `Step 2`, in the tab `Wizard`, select `Output` under `Topics` and untick **LaTeX** option unless desired.
- In `Step 2`, in the tab `Expert`, select `HTML` under `Topics` and tick`USE_MATHJAX` option to display mathematical formulas.
- In `Step 2`, in the tab `Expert`, select `Input` under `Topics`, go to `FILE_PATTERNS` and remove all extensions except _\*.c, \*.cc, \*.cpp, \*.c++, \*.h, \*.hh, \*.hpp, \*.h++_ by selecting them and clicking the minus button. Look below for `EXCLUDE_PATTERNS` and add _tests_ and _doc_ by clicking the plus button. In the `EXCLUDE` list right above you should also add any build folder which is inside the source code directory. Finally add the following words to the `EXCLUDE_SYMBOLS` list:
  - STATE_DEFINE
  - GUARD_DEFINE
  - ENTRY_DEFINE
  - EXIT_DEFINE
- To test your configuration, in `Step 2`, go to tab `Run` and click `Run doxygen` button. In the integrated console you will see your documentation building up. When it is done, click on `Show HTML output` and you will be prompted to a web page on your favourite browser, which you can navigate on.
- Close the window. You will be asked to save the _Doxyfile_ in the project directory. Do it as it suggests and you are done!
Obviously, you can edit your configuration file with your favourite text editor in a later stage.

### Setup Doxygen in your Qt project

Assuming the code you want to document already belongs to a Qt project, open it with _QtCreator_ and go to _Projects_ tab. In the **Build Settings**, under the **Build Steps** section, click on `Add Build Step` button and select `Custom Process Step`.
In the `Command` field type `doxygen`, while in the `Arguments` field you need to write the absolute path to the _Doxyfile_ we just created, i.e. _$MYPROJECTPATH/Doxyfile_.

That's it! On your next build, _Doxygen_ will parse all the files present in the project, look for documented lines within the code and generate an `html` file with all the information found nicely arranged.
The main file you want to open is `index.html`, which you will find in the specified _doc_ folder.

Note that _Doxygen_ may generates a lots of warning, especially the first times you use it, because your code is probably not yet compliant with all the formatting rules. These warnings do not affect the performance or outcome of your application, but can hide serious ones generated because of specific user-specific errors, unrelated to documentation. Our suggestion is to lose some time at the beginning to solve them one-by-one, and add compliant comments while developing new code right away as you move forward. It is a good practise which makes life easier for you and especially for your future and present colleagues! :)

## Authors

- **Edoardo Idà** - _Initial work_ - edoardo.ida2@unibo.it
- **Simone Comari** - simone.comari2@unibo.it

## Maintainers

- **Simone Comari** - simone.comari2@unibo.it
