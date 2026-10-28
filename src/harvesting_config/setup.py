from setuptools import setup

setup(
    name='harvesting_config',
    version='0.1.0',
    packages=['harvesting_config'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/harvesting_config']),
        ('share/harvesting_config', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harvesting maintainers',
    maintainer_email='davidnie0418@gmail.com',
    description='Hardware-independent UR10e deployment configuration',
    license='TODO: License declaration',
)
