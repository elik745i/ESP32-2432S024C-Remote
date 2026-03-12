## v0.2.9

This release includes the full UI and runtime changes added after `v0.2.6`.

### UI and Navigation

- Added press-and-hold drag reordering across menus and many submenu lists, with saved order persisted across reboots.
- Drag mode now has clearer visual feedback: delayed activation, lifted/enlarged item state, smoother slot targeting, reduced jumping, and click suppression after drop.
- Added global `Flat Buttons` and `3D Buttons` styles, with persisted selection and selector colors.
- Stateful buttons now show stronger concave pressed styling when active, with darker green active shading for remembered toggles.
- Added built-in icons in front of button labels across the main UI.
- Improved swipe-back reliability, especially on dense vertical button menus and scrollable lists.
- Reduced gesture conflicts between back-swipe, scrolling, button clicks, and reorder drag.

### Connectivity and Recovery

- Replaced the old `Web Recovery` home action with `AP Mode`.
- `AP Mode` can now be toggled from the home screen without deleting saved networks.
- Session AP mode is temporary only: it does not persist across reboot, and the device returns to normal saved-Wi-Fi behavior on the next boot.
- Pressing `AP Mode` again while active now exits AP mode and retries connection to saved Wi-Fi.
- Fixed AP mode UI state mismatches so the button reflects the actual session AP state correctly.

### Visual Updates

- Replaced the unreliable airplane glyph approach with an embedded image-based airplane indicator.
- Moved the airplane toggle to the main menu and improved its active-state visual treatment.
- Added battery, Wi-Fi, and lighting bars to the `Info` screen.
- Enlarged the `Snake 3D` start popup and lightened/optimized parts of the `Snake 3D` floor rendering.

### Performance and Stability

- Moved LVGL allocation on the `ESP32-S3-3248S035-N16R8` build to a PSRAM-first allocator to reduce internal SRAM pressure on heavy menus.
- Reduced unnecessary top-bar redraws and repeated label/bar refresh work in several high-frequency UI paths.
- Hardened screen transition and touch-state handling when moving between menus and swiping back.
- Fixed repeated LVGL config include warnings in the build by making the project include path explicit.

### Firmware Binaries

- `esp32-2432s024c-v0.2.9.bin`
- `esp32-3248s035-v0.2.9.bin`
- `esp32-s3-3248s035-n16r8-v0.2.9.bin`
