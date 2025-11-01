# Boxygo

**Boxygo** is an innovative solution for residential complexes – an autonomous parcel delivery robot that is part of an integrated package delivery system. It enables contactless delivery of parcels directly to the residents' doorsteps.

## Key Features
- **Autonomy:** The robot navigates independently within residential areas.
- **Convenience:** Secure and effortless parcel delivery without the involvement of traditional couriers.
- **Modernity:** A pioneering approach to logistics and parcel handling in gated communities.

Boxygo sets a new standard in deliveries, benefiting both residents and service providers.

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

# Docker

## Usefull commands

**Create Dev Docker Image**
`sudo docker build -f docker/Dockerfile.dev -t boxygo/dev:latest`

**Compose Dev Docker Image**
`sudo docker-compose -f docker/compose.dev.yml run --rm dev`

## Working on dev container

**If you want to use Gazbo or Rviz (any GUI) you have give acces 11 by following command in terminal**
`xhost +local:`




