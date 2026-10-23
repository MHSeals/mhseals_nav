import pytest

from ros_domain import require_isolated_domain


@pytest.mark.parametrize("domain", ["1", "42", "231", "232"])
def test_any_isolated_domain_is_accepted(domain):
    assert require_isolated_domain({"ROS_DOMAIN_ID": domain}) == int(domain)


@pytest.mark.parametrize("domain", [None, "", "0", "233", "abc", "-3"])
def test_default_unset_or_out_of_range_domain_is_rejected(domain):
    with pytest.raises(AssertionError):
        require_isolated_domain({} if domain is None else {"ROS_DOMAIN_ID": domain})
