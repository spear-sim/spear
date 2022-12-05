![SPEAR](docs/images/teaser_web.jpg "SPEAR")

# SPEAR

Interactive simulators are becoming powerful tools for training embodied agents, but existing simulators suffer from limited content diversity, physical interactivity, and visual fidelity. We address these limitations by introducing SPEAR, a Simulator for Photorealistic Embodied AI Research. To create our simulator, we worked closely with a team of professional artists for over a year to construct 300 unique virtual indoor environments with 2,566 unique rooms and 17,234 unique objects that can be manipulated individually. Each of our environments features detailed geometry, photorealistic materials, and a unique floor plan and object layout designed by a professional artist, i.e., we do not rely on remixing existing layouts to create additional content. Our environments are implemented as Unreal Engine assets, and we provide an OpenAI Gym interface for interacting with the environments via Python.

The SPEAR code is licensed under is licensed under an [MIT License](http://opensource.org/licenses/MIT), and the SPEAR assets are licensed under a permissive license that permits academic use. See [LICENSE.txt](LICENSE.txt) for the license that applies to our code, and [LICENSE_ASSETS.txt](LICENSE_ASSETS.txt) for the license that applies to our assets.

## Citation

If you find SPEAR useful in your research, please cite this repository as follows:

```
@misc{roberts:2022,
    author       = {Mike Roberts AND Quentin Leboutet AND Rachith Prakash AND Renhan Wang AND
                    Hailin Zhang AND Rui Tang AND Marti Ferragut AND Stefan Leutenegger AND
                    Stephan R. Richter AND Vladlen Koltun AND Matthias M{\"u}ller AND German Ros},
    title        = {{SPEAR}: {A} Simulator for Photorealistic Embodied AI Research},
    howpublished = {\url{http://github.com/isl-org/spear}},
    year         = {2022},
}
```
