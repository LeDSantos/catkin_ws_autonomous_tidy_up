## Instructions to prepare the computer to run a Singularity image

Download the image from here: https://drive.google.com/file/d/1ejx5BQBuqmfi5IweD9WeJYN6rL5RS6F7/view?usp=sharing

As the text in the master thesis and paper describes, the tests with the real robot and the simulated tests used different computers. The image was built in the real-world test computer using Singularity version 3.5.3. But, this version has a bug that doesn't allow to run multiple terminals simultaneously after many connections with the image.

So, here are the steps that were used to prepare the simulation tests computer to use the singularity image:

- The computer has Ubuntu 20.04.06 LTS, so the singularity-ce version 3.11.4-focal were installed, downloaded from [here](https://github.com/sylabs/singularity/releases/download/v3.11.4/singularity-ce_3.11.4-focal_amd64.deb). Other releases: https://github.com/sylabs/singularity/releases

- Verify the adequate driver for the NVIDIA GPU:

```sh
$ ubuntu-drivers devices
```

- Go to Software & Updates > Additional Drivers.

- Select the recomended driver and Apply Changes. This can take a while.

- Restart the computer and verify if the GPU name is on Settings. The driver should create the folder home/.nv, that will be used by the singularity image.