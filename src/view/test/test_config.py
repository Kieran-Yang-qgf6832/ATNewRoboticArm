import pytest

from view import config


def test_defaults_are_valid():
    cfg, problems = config.config_from_values({})
    assert problems == []
    assert cfg == config.DetectorConfig()


def test_rejects_out_of_range():
    with pytest.raises(config.ParameterError):
        config.validate('h_low', 200)
    with pytest.raises(config.ParameterError):
        config.validate('s_high', -1)
    with pytest.raises(config.ParameterError):
        config.validate('min_area', 0)


def test_kernel_must_be_non_negative_odd():
    with pytest.raises(config.ParameterError):
        config.validate('morph_kernel', 4)
    with pytest.raises(config.ParameterError):
        config.validate('morph_kernel', -3)
    assert config.validate('morph_kernel', 0) == 0
    assert config.validate('morph_kernel', 3) == 3


def test_rejects_wrong_type():
    with pytest.raises(config.ParameterError):
        config.validate('h_low', 3.5)
    with pytest.raises(config.ParameterError):
        config.validate('h_low', True)
    with pytest.raises(config.ParameterError):
        config.validate('display', 1)


def test_int_accepted_for_float_field():
    assert config.validate('square_size_mm', 350) == 350.0


def test_unknown_parameter_is_rejected():
    with pytest.raises(config.ParameterError):
        config.validate('color_topic', 'x')


def test_bad_startup_value_falls_back_to_default():
    cfg, problems = config.config_from_values({'h_low': 999, 'morph_kernel': 3})
    assert cfg.h_low == config.FIELDS['h_low'].default
    assert cfg.morph_kernel == 3
    assert len(problems) == 1


def test_overrides_are_all_or_nothing():
    cfg = config.DetectorConfig()
    with pytest.raises(config.ParameterError):
        config.apply_overrides(cfg, {'h_low': 100, 'morph_kernel': 4})
    assert cfg.h_low == 170

    updated = config.apply_overrides(cfg, {'h_low': 100})
    assert updated.h_low == 100
    assert cfg.h_low == 170
