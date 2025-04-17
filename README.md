# WebGPU Compute Playground

![WebGPU Supported](https://img.shields.io/badge/WebGPU-Native%20C++-blue)
![C++](https://img.shields.io/badge/C++-blueviolet?logo=c%2B%2B&logoColor=white)
![CMake](https://img.shields.io/badge/CMake-brightgreen?logo=cmake&logoColor=white)
![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20Linux-lightgrey.svg)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

A lightweight playground for experimenting with WebGPU compute shaders using the native C++ API. This project builds on top of the Dawn project, which is an open-source implementation of the WebGPU standard. It provides a basic framework to write, build, and run WebGPU compute pipelines directly from C++, allowing for low-level control and experimentation without the browser environment.

## Getting Started

1. **Clone the Repository:**

    ```shell
    git clone https://github.com/daijh/wgpu_compute_playground.git 
    cd webgpu_compute_playground
    ```

2. **Initialize/Update Submodules:**

    ```shell
    git submodule update --init
    ```

3. **Build the Project:**

    ```shell
    cmake -S . -B build
    cmake --build build -j8
    ```

4. **Run:**

    ```shell
    # Windows
    build\wgpu\Debug\basic_compute.exe

    # Linux
    build/wgpu/basic_compute
    ```

## Contributing

Contributions are welcome! If you have any ideas for improvements, new features, or bug fixes, feel free to open an issue or submit a pull request.

## License

This project is licensed under the BSD 3-Clause "New" or "Revised" License. See the `LICENSE` file for more information.
