#!/bin/sh
cd cpp
python3 -m mkdocs build
cd ..
cd py
python3 -m mkdocs build
cd ..
rm -rf docs/*
cp -r cpp/site_build/* docs/
cp -r py/site_build docs/py/
git add .
git commit -m "rebuild"
git push