from setuptools import setup, Extension

package_name = 'dca1000'

module = Extension('circular_buffer', 
                   sources = ['dca1000/circular_buffer.c'])

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    ext_modules = [module],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='plaba',
    maintainer_email='plaba417@students.bju.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dca1000 = dca1000.raw_mmwave_node:main'
        ],
    },
)
