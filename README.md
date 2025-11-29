# Boxygo

**Boxygo** is an innovative solution for residential complexes – an autonomous parcel delivery robot that is part of an integrated package delivery system. It enables contactless delivery of parcels directly to the residents' doorsteps.

# Configure environment

1. Clone repo into this directory: `git clone https://github.com/BoxyGo/BoxyGo.git ~/workspaces/BoxyGo`
2. Go to workspace: `cd ~/workspaces/BoxyGo`
3. Run script: `./setup_dev_environment.sh`
4. Run script: `./run_dev_container.sh`
5. If run was successfull you can start development

# Aliases
Aliases added to container with `configure_dev_container.sh`
`b` - build workspace
`k` - run teleop keyboard
# Moteus configuration:

**Wheels config:**
`conf set aux1.spi.mode 1`      
`conf set aux1.pins.0.mode 6`  
`conf set aux1.pins.0.pull 1`  
`conf set aux1.pins.2.mode 6`   
`conf set aux1.pins.2.pull 1`   
`conf set aux1.pins.3.mode 6` 
`conf set aux1.pins.3.pull 1`  
`conf set aux1.hall.enabled 1` 
`conf set motor_position.sources.0.type 4`

**Wheels calibration command:**
`python3 -m moteus.moteus_tool --target 1 --calibrate --cal-hall --call-motor-poles 30`

**Write config to wheel command:**
`python3 -m moteus.moteus_tool --target 1 --write-config FILE`




