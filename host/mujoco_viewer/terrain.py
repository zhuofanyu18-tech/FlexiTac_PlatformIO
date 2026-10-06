"""Continuous "mountain" surface for a rows x cols tactile frame.

The sensor matrix is padded with one zero cell, upsampled with bicubic
interpolation and Gaussian-blurred, then written into a MuJoCo height field.
A 2D texture of the same resolution is colored by height, so a harder press
is both taller and a different color.
"""

import cv2
import mujoco
import numpy as np

COLORMAPS = ("turbo", "jet", "viridis", "inferno", "plasma", "hot")


def scene_xml(nrow, ncol, half_x, half_y, height, base, offscreen=(1280, 720)):
    """MuJoCo scene: one hfield (shape) + one 2D texture (color) on a floor."""
    floor = 3 * max(half_x, half_y)
    return f"""
<mujoco model="flexitac_terrain">
  <visual>
    <global offwidth="{offscreen[0]}" offheight="{offscreen[1]}"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.45 0.45 0.45" specular="0.1 0.1 0.1"/>
    <map znear="0.001"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.25 0.35 0.5" rgb2="0.02 0.02 0.05" width="512" height="3072"/>
    <texture name="grid" type="2d" builtin="checker" mark="edge" rgb1="0.12 0.14 0.17" rgb2="0.09 0.1 0.13"
             markrgb="0.3 0.3 0.3" width="300" height="300"/>
    <material name="grid" texture="grid" texuniform="true" texrepeat="40 40"/>
    <hfield name="surface" nrow="{nrow}" ncol="{ncol}" size="{half_x} {half_y} {height} {base}"/>
    <texture name="surface" type="2d" builtin="flat" rgb1="0 0 0" width="{ncol}" height="{nrow}"/>
    <material name="surface" texture="surface" specular="0.35" shininess="0.5"/>
  </asset>
  <worldbody>
    <light directional="true" pos="0 0 1" dir="-0.3 0.4 -1" diffuse="0.6 0.6 0.6" castshadow="false"/>
    <geom name="floor" type="plane" size="{floor} {floor} 0.01" pos="0 0 {-base - 0.001}" material="grid"/>
    <geom name="surface" type="hfield" hfield="surface" material="surface" contype="0" conaffinity="0"/>
  </worldbody>
</mujoco>
"""


class Terrain:
    """Owns the MuJoCo model and converts 0..1 sensor intensity to terrain."""

    def __init__(self, rows, cols, pitch=0.005, height=0.03, upsample=8, blur=0.7, cmap="turbo"):
        self.rows, self.cols = rows, cols
        self.upsample = upsample
        self.blur = blur * upsample  # sigma in upsampled pixels
        self.nrow, self.ncol = (rows + 2) * upsample, (cols + 2) * upsample
        self.half_x, self.half_y = (cols + 2) * pitch / 2, (rows + 2) * pitch / 2
        self.height = height
        self.colormap = getattr(cv2, f"COLORMAP_{cmap.upper()}")
        xml = scene_xml(self.nrow, self.ncol, self.half_x, self.half_y, height, pitch / 2)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.hfield_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_HFIELD, "surface")
        self.texture_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_TEXTURE, "surface")
        self.hfield_slice = slice(self.model.hfield_adr[self.hfield_id],
                                  self.model.hfield_adr[self.hfield_id] + self.nrow * self.ncol)
        self.channels = int(self.model.tex_nchannel[self.texture_id])
        start = self.model.tex_adr[self.texture_id]
        self.texture_slice = slice(start, start + self.nrow * self.ncol * self.channels)
        self.peak = 0.0
        self.apply(np.zeros((rows, cols), dtype=np.float32))
        mujoco.mj_forward(self.model, self.data)

    def surface(self, intensity):
        """rows x cols in 0..1 -> smooth (nrow, ncol) map in 0..1; image row 0 = sensor row 0."""
        padded = np.pad(np.asarray(intensity, dtype=np.float32), 1)
        smooth = cv2.resize(padded, (self.ncol, self.nrow), interpolation=cv2.INTER_CUBIC)
        if self.blur > 0:
            smooth = cv2.GaussianBlur(smooth, (0, 0), self.blur)
        return np.clip(smooth, 0, 1)

    def apply(self, intensity):
        """Write heights and colors into the model (call viewer.update_* afterwards)."""
        smooth = self.surface(intensity)
        self.peak = float(smooth.max())
        # hfield row 0 is the -Y edge; flip so sensor row 0 is the far (+Y) edge.
        self.model.hfield_data[self.hfield_slice] = smooth[::-1].ravel()
        # Texture rows run from +Y to -Y, i.e. the same order as the sensor image.
        bgr = cv2.applyColorMap((smooth * 255).astype(np.uint8), self.colormap)
        rgb = bgr[..., ::-1]
        if self.channels == 4:
            rgb = np.dstack([rgb, np.full(smooth.shape, 255, np.uint8)])
        self.model.tex_data[self.texture_slice] = rgb.ravel()

    def set_camera(self, cam):
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = (0, 0, self.height * 0.2)
        cam.distance = 2.2 * self.half_x
        cam.azimuth = 90
        cam.elevation = -40
