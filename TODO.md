These are TODOs to finish the next export

- [ ] Update `CMakeLists.txt` with all of the new added source files
  - [ ] One at a time (add to git when added here)
  - [ ] First, add all files that compile as-is
  - [ ] Next, add files that require more dependencies (add those dependencies)
  - [ ] Add those dependencies to the `CMakeLists.txt` file and also to the
        `README.md` file
  - [ ] Finally, we will have a list of files that need to be modified to get
        them to compile (i.e., files that break the build)

- [ ] Modify files that break the build
  - [x] Modify python bindings for broken code
  - [ ] Modify files in `src/vistendon` to remove need for collision types

- [ ] Update the `README.md` file
  - [ ] Each new added file and section
  - [ ] Updated python bindings
  - [ ] Docker

- [ ] Copy more tests over

- [ ] Test out build in Docker

- [ ] Remove extraneous stuff
  - [ ] `tip-control`: remove our own implementation of dampled least squares
