from tools.validate_antiwheelie import case_transmission


def test_reject_coarse_is_pinned_to_the_chain_whose_instability_it_tests():
    assert case_transmission('reject_coarse', 'ideal_mid_drive') == 'elastic_chain'
    assert case_transmission('reject_coarse', 'geometric_ideal_mid_drive') == 'elastic_chain'
    assert case_transmission('wheelie', 'ideal_mid_drive') == 'ideal_mid_drive'
    assert case_transmission('limited', 'elastic_chain') == 'elastic_chain'
