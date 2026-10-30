"""centerToFront must come from the platform CarSpecs, not an interface override."""

import pytest

from opendbc.car.car_helpers import interfaces
from opendbc.car.mazda.values import CAR


@pytest.mark.parametrize("platform", list(CAR))
def test_center_to_front_uses_platform_specs(platform):
  # Ratios are per platform (MoreTore GEN2 values include 0.38); the interface must not override them.
  specs = platform.config.specs

  CP = interfaces[platform].get_params(platform, {bus: {} for bus in range(8)}, [],
                                       alpha_long=False, is_release=False, docs=False)
  assert CP.centerToFront == pytest.approx(CP.wheelbase * specs.centerToFrontRatio)
