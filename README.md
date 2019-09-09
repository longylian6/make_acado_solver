make_acado_solver
======
This code is a part of https://github.com/ethz-asl/mav_control_rw.git. 

Author: Oskar Ljungqvist
Date: 2017-11-23
Description:

This folder contains code to Automatically generate an MPC controller using ACADO toolkit.

The file *nmpc_solver_setup.cpp* is what defines the MPC controller and it is this you should change in order the change the controller. Then build/install it using the explanation below. As an example it easy to change the prediction horizon if that is prefered.

Prerequests: 
------

ACADO toolkit needs to be properly installed. Follow the instructions here: http://acado.github.io/install_linux.html

* In your ~/.bashrc add the line:
```sh
source [pathToAcado]/ACADOtoolkit/build/acado_env.sh
```

* Build/Install. From this folder do:
```sh
cd [yourDirectory]/make_acado_solver
mkdir build
cd build
cmake ..
make
```
* The executable is now placed in the folder ../solver. Execute it by running the following commands:
```sh
cd ../solver
./nmpc_solver_setup
```
Then the controller is generated on *OCPexport* folder.




# 中文简单讲解
* 下载安装acado

* 增加环境变量，在 *~/.bashrc* 增加一句
```sh
export [你的路径]/ACADOtoolkit/build/acado_env.sh
```

* 构建求解器，在./nmpc_solver_setup.cpp上构建你的求解器

* 生成求解器
```sh
cd [你的路径]/make_acado_solver
mkdir -p build
cd build
cmake ..
make 
cd ../solver
./nmpc_solver_setup
```
然后会在*OCPexport*文件里生成你的解析器了

* 在Clion上使用求解器，使用解析器的代码在./test_on_clion/main.cpp

* 每次改动求解器后，要记得删除./test_on_clion/solver/OCPexport，将新的./solver/OCPexport文件夹替换原来的文件夹
