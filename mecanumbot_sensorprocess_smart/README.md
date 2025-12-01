# Smart sensor processer nodes for the mecanumbot package

In this folder, nodes are situated which extract information from the mecanumbot's on-board sensors

## Nodes:

#### External nodes:

- Package containing the dr_spaam ros1 syntax and model: https://github.com/VisualComputingInstitute/DR-SPAAM-Detector/tree/master
    install steps: #TODO -make pretty 
        cd [intended storing folder]
        git clone https://github.com/VisualComputingInstitute/DR-SPAAM-Detector.git
        cd []/DR-SPAAM-Detector/dr_spaam
        python setup.py install
        other requrements:
                torch: pip install torch
- source of the neural network: https://robotics.upo.es/datasets/frog/laser2d_people/