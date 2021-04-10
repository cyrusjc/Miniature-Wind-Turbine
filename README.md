# Miniature-Wind-Turbine
ELEC 391 Design Project. Miniature Wind Turbine, hand wound generator, 3D printed turbine blades, with a buck converter for MPPT, uploaded onto a cloud server, built with lego.


![image](https://user-images.githubusercontent.com/32754336/114273239-1b951580-99ce-11eb-90ea-7790d9f49d57.png)

fig.1 Windmill in question

For the PCB, Altium designer was used, and ordered off of an online PCB manufacturer in china. 

![image](https://user-images.githubusercontent.com/32754336/114273364-99592100-99ce-11eb-9cb8-b77d54caa7c2.png)

fig.2 PCB with Boost Converter using 1 diode and 1 mosfet topology.

For the MPPT algo, we used P&O which is state dependent. checks previous power and duty cycle. the flow chart of the logic is listed below:

![image](https://user-images.githubusercontent.com/32754336/114273725-1df86f00-99d0-11eb-922b-6fbeccd57451.png)

The MPPT algo is modeled on matLAB using a special function block. 
