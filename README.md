# URP Dynamic Diffuse Global Illumination

## This repo is in prototyping phase and is currently not working. Come back later.

An implementation of the ["Dynamic Diffuse Global Illumination with Ray-Traced Irradiance Fields" paper](https://jcgt.org/published/0008/02/01/), as well as [RTX-DDGI repo](https://github.com/NVIDIAGameWorks/RTXGI-DDGI).

Many improvements were borrowed from [Timberdoodle's](https://github.com/Sunset-Flock/Timberdoodle/) PGI system.

## TODO
- Rendering the probes debug view, reading from the texture
- Implementing hardware agnostic-raytracing by reusing existing BVH8-based [pathtracing solution] (https://github.com/Pjbomb2/TrueTrace-Unity-Pathtracer)
- Probe visibility generation
- Probe radiance generation
- Probe sampling
- Cascaded probe system
- Sparse probe system
  - Detaching physical probe location from location in memory
  - Allow probes to be "allocated" on the GPU as necessary
- Lazy probe updating
  - Figure out which probes are in view and prioritize probes on screen.
- Allow for script-driven probe freezing
  - If you know ahead of time a certain region in space will be static for a given amount of time, freeze certain probes.
  - The trade off is that probes can't be deallocated if they aren't in view, but at the cost of not updating the  
