"""ROS 2 ament 설치 메타데이터. 런타임 파일만 포함한다."""
from pathlib import Path
import sys
from setuptools import find_packages, setup
from setuptools.dist import Distribution

root = Path(__file__).parent
native_name = ("smartcart_uwb_parser.dll" if sys.platform == "win32" else
               "libsmartcart_uwb_parser.dylib" if sys.platform == "darwin" else
               "libsmartcart_uwb_parser.so")


class RuntimeDistribution(Distribution):
    """네이티브 라이브러리가 포함된 wheel을 다른 플랫폼용으로 표시하지 않는다."""
    def has_ext_modules(self):
        return (root / "smartcart_uwb" / "lib" / native_name).is_file()


data = [("share/ament_index/resource_index/packages", ["resource/smartcart_sensing"]),
        ("share/smartcart_sensing", ["package.xml", "LICENSE"]),
        ("share/smartcart_sensing/launch", ["launch/sensing.launch.py", "launch/dual_uwb.launch.py",
                                         "launch/tracking.launch.py"]),
        ("share/smartcart_sensing/config", ["config/dual_uwb.d5.example.json",
                                          "config/uwb_estimation.example.json",
                                          "config/target_tracking.example.json",
                                          "config/field_acceptance.example.json",
                                          "config/uwb_estimation.synthetic.json",
                                          "config/target_tracking.synthetic.json"]),
        ("share/smartcart_sensing/samples", ["samples/field_acceptance.synthetic.jsonl",
                                             "samples/target_tracking.synthetic.jsonl",
                                             "samples/target_tracking.truth.json"])]
for folder in sorted((root / "web").rglob("*")):
    if folder.is_dir():
        files = [str(p.relative_to(root)) for p in sorted(folder.iterdir()) if p.is_file()]
        if files:
            data.append(("share/smartcart_sensing/" + str(folder.relative_to(root)).replace("\\", "/"), files))
data.append(("share/smartcart_sensing/web", ["web/index.html"]))

setup(name="smartcart_sensing", version="0.1.0", packages=find_packages(exclude=("tests",)),
      data_files=data, install_requires=["setuptools"], zip_safe=False,
      description="스마트카트 LiDAR·UWB 읽기 전용 센싱", license="MIT",
      distclass=RuntimeDistribution,
      package_data={"smartcart_uwb": ["lib/" + native_name]},
      entry_points={"console_scripts": ["smartcart-sensing = smartcart_sensing.app:main",
                                         "smartcart-uwb = smartcart_uwb.app:main",
                                         "smartcart-uwb-ros = smartcart_uwb.ros_adapter:main",
                                         "smartcart-target-replay = smartcart_tracking.cli:main",
                                         "smartcart-field = smartcart_field.cli:main",
                                         "smartcart-cart = smartcart_runner.app:main"]})
