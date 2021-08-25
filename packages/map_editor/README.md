# mse_visual_map_editor_for_duckietown
App for editing duckietown maps.

*Note: Editing 0-layer (tile layer) of structure [Confluence. Maps in Duckietown World](https://ethidsc.atlassian.net/wiki/spaces/DS/pages/448593943/Design+Document+Maps+in+Duckietown+World#2.1.-Structure)*

## Getting started
### Requirements
* python 3.5+
* PyQt5 5.15
``` bash
# Installing python-dependencies
python3 -m pip install -r requirements.txt
```
### Run
``` bash
# Run app
python3 main.py
```
`./maps` contains examples of maps.

## Multi language support
[Wiki: Multi language support](https://github.com/moevm/mse_visual_map_editor_for_duckietown/wiki/Multi-language-support)

## IN PROGRESS...
## Building a single-package application for a specific OS

### Requirements
```bash
# Installing python-dependencies
python3 -m pip install -r requirements.txt
```

## Building and running the application
### Build
```bash
pyinstaller --onefile --noconsole main.py
```
### Run
```bash
# Move file to repository root directory
mv dist/main main
# Run app
./main
```
