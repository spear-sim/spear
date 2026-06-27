# Control the Procedural City Generator

In this example application, we demonstrate how to control Unreal's procedural city generator. We reshape the generator's `City_Shape` spline to spell `SPEAR`, regenerate the city, and then fly the editor viewport camera through the result.

Open `SpearSim.uproject` in the Unreal Editor, open the `/PCGPrimitives/Examples/City/City_Generator_steps` map, and wait for the map to fully load. Then we press play in the editor and wait for the Unreal simulation to load and warm up. Once the simulation is fully loaded and warmed up, we are ready to control the city generator via SPEAR.

```console
python run.py
```

You should see the city's outline reshape itself to spell `SPEAR`, the buildings and roads regenerate to fill the new outline, and then the viewport camera fly through the city before pulling up to reveal the letters from above.
