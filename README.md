Need certain IDF version to build the project.

In your IDF directory.

```bash
cd $IDF_PATH
# I assume your origin is 
# https://github.com/espressif/esp-idf.git
# or v4.4.6
git fetch origin release/v4.4
git checkout release/v4.4
git submodule update --init --recursive
./install.sh
source export.sh
```

```
-DIDF_MAINTAINER=1
```
