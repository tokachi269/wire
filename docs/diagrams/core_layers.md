# core_layers.md

## Core Layer Map

```mermaid
flowchart TD
  api["Public Model / API
  core/include/wire/core/*.hpp"]

  state["State / Entity Ownership
  core/src/state"]
  generation["Generation / Workflow
  core/src/generation"]
  validation["Validation
  core/src/validation"]
  derived["Direct Derived Output
  span layout / geom / draw caches"]
  tests["Tests
  core/tests"]
  inspection["Inspection Surface
  core/include/wire/core/inspection.hpp
  core/src/state/inspection.cpp"]
  viewer["Viewer
  viewer/src"]

  api --> state
  api --> generation
  api --> validation

  generation --> state
  generation --> derived
  state --> derived
  derived --> validation

  inspection --> api
  inspection --> state
  inspection --> derived

  tests --> api
  tests --> state
  tests --> generation
  tests --> derived
  tests --> validation

  viewer --> api
  viewer --> inspection
```

## High-Difference Curve Path

```mermaid
flowchart LR
  rules["SpanLayoutRules
  saved decision"]
  layout["SpanLayoutEntry
  endpoint placement"]
  detail["detail_curve
  direct curve"]
  cache["curve cache / inspection"]
  viewer["viewer
  consume only"]

  rules --> layout --> detail --> cache --> viewer
```
