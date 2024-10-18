meta:
  id: hr_data
  title: HR Data
  endian: be
doc: |
  HR data is a pair of band id and heart rate. Nothing fancy.
seq:
  - id: hr_pair
    type: pair

types:
  pair:
    seq:
      - id: num_band_name
        type: u1
      - id: band_name
        type: str
        size: num_band_name
        encoding: UTF-8
      - id: heart_rate
        type: u1
