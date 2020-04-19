from setuptools import setup

setup(name='ormope',
      version='0.1',
      description='OpenRAVE-based Motion Planning environments',
      url='https://github.com/rjgpinel/ormope',
      author='Ricardo Garcia',
      author_email='rjgpinel@gmail.com',
      install_requires=["gym"]
      packages=find_packages(),
)
