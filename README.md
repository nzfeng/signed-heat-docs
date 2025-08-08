To edit the docs, install `mkdocs-material` via 
```
pip install mkdocs-material
```

To preview changes to the docs,
```
mkdocs serve
```

After editing, build a static site with
```
mkdocs build
```

To publish, run
```
sh build_docs.sh
```
to build the static site, and commit and push the changes. 

Deploy to GitHub Pages by using the [GitHub web interface](https://docs.github.com/en/pages/getting-started-with-github-pages/configuring-a-publishing-source-for-your-github-pages-site) (could also be automated)