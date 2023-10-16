# How to use core-dev artifact

## Usage

Build docker image

```bash
make build
```

Run container

```bash
make run
```

Join container

```bash
make join
```

Stop container

```bash
make stop
```

Remove container

```bash
make rm
```

## Work inside container

*First of all you need to run and join a container.*


Build ros workspace
```bash
ws_build.sh --all
```

Build special folder
```bash
ws_build.sh --folder
```

Build this folder
```bash
ws_build.sh --this
```

Run simulator
```bash
ws_run.sh --sim
```

Run navigation
```bash
ws_run.sh --navigation
```

Run rosbridge
```bash
ws_run.sh --rosbridge
```

Run gui
```bash
ws_run.sh --gui
```

Run simulator, planning and control
```bash
ws_run.sh --full-stack
```
