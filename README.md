![teaser](https://github.com/spear-sim/spear/assets/2341965/6db5ff27-4d12-4097-8f7e-e31bd479844e)

# SPEAR: A Simulator for Photorealistic Embodied AI Research

Interactive simulators are becoming powerful tools for training embodied agents, but existing simulators suffer from limited content diversity, physical interactivity, and visual fidelity. We address these limitations by introducing SPEAR: A Simulator for Photorealistic Embodied AI Research. To create our simulator, we worked closely with a team of professional artists for over a year to construct 300 unique virtual indoor environments with 2,566 unique rooms and 17,234 unique objects that can be manipulated individually. Each of our environments features detailed geometry, photorealistic materials, and a unique floor plan and object layout designed by a professional artist, i.e., we do not rely on remixing existing layouts to create additional content. Our environments are implemented as Unreal Engine assets, and we provide an OpenAI Gym interface for interacting with the environments via Python.

The SPEAR code is released under an [MIT License](LICENSE.txt), and the SPEAR assets are released under various [licenses](#licenses) that permit academic use.

## Citation

If you find SPEAR useful in your research, please cite this repository as follows:

```
@misc{spear,
    author       = {Mike Roberts AND Rachith Prakash AND Renhan Wang AND Quentin Leboutet AND
                    Stephan R. Richter AND Stefan Leutenegger AND Rui Tang AND Matthias
                    M{\"u}ller AND German Ros AND Vladlen Koltun},
    title        = {{SPEAR}: {A} Simulator for Photorealistic Embodied AI Research},
    howpublished = {\url{http://github.com/spear-sim/spear}}
}
```

## Getting Started

### Minimum and recommended system specifications

Minimum and recommended system specifications for the Unreal Engine are given [here](https://docs.unrealengine.com/5.2/en-US/hardware-and-software-specifications-for-unreal-engine).

### Precompiled binaries

See our latest [release notes](https://github.com/spear-sim/spear/releases/tag/v0.5.0) for download links. The easiest way to start working with SPEAR is to download a precompiled binary for your platform. Our precompiled binaries come pre-packaged with the scene pictured above. You can start interactively navigating around this scene with the keyboard and mouse simply by running the downloaded binary with no additional arguments.

### Documentation

- Our [Getting Started](docs/getting_started.md) tutorial explains how to interact with multiple scenes and our Python interface.
- Our [Building SpearSim](docs/building_spearsim.md) tutorial explains how to build from source.
- Our [Importing and Exporting Assets](docs/importing_and_exporting_assets.md) tutorial explains how to import and export assets.
- Our [Coding Guidelines](docs/coding_guidelines.md) document describes our coding standard.
- Our [Contribution Guidelines](CONTRIBUTING.md) document contains information on how to contribute effectively.

## Licenses

#### Code

- The code in this repository is licensed under an [MIT License](LICENSE.txt).
- The licenses for all of our third-party code dependencies are given [here](ACKNOWLEDGMENTS.txt).

#### Assets

- The `apartment`, `debug`, and `warehouse` scenes are licensed under a [CC0 License](http://creativecommons.org/publicdomain/zero/1.0).
- The OpenBot and Fetch assets in this repository are licensed under a [CC0 License](http://creativecommons.org/publicdomain/zero/1.0).
- The license for the `kujiale` scenes is given [here](LICENSE_KUJIALE.txt).
- The license for the `StarterContent` assets referenced in the `debug` scenes is given [here](https://www.unrealengine.com/en-US/eula/unreal).

## Acknowledgements

From 2021 to 2024, SPEAR was developed with generous support from the [Intelligent Systems Lab](https://github.com/isl-org) at [Intel](https://www.intel.com) and [Kujiale](https://www.kujiale.com). Beginning in 2024, SPEAR is being developed by an independent consortium with generous support from Kujiale.
