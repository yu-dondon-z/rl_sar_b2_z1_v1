import sys
from pathlib import Path
import unittest

import numpy as np
import torch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/diagnostics'))
from analyze_vd6_actor_feedback import integrated_change, summarize_contributions, OUT, GROUPS


class FeedbackAttributionTests(unittest.TestCase):
    def test_linear_actor_exact_feature_contributions_and_scaling(self):
        torch.manual_seed(17)
        model=torch.nn.Linear(70,18).double()
        before=torch.randn(7,70,dtype=torch.float64)
        after=torch.randn(7,70,dtype=torch.float64)
        actual=integrated_change(model,before,after,8)
        expected=(.25*(after-before)[:,None,:]*model.weight[OUT][None,:,:]).detach().numpy()
        np.testing.assert_allclose(actual,expected,atol=1e-12)

    def test_nonlinear_path_reconstructs_delta_and_zero_input_changes_stay_zero(self):
        torch.manual_seed(18)
        model=torch.nn.Sequential(torch.nn.Linear(70,32),torch.nn.ELU(),torch.nn.Linear(32,18)).double()
        before=torch.randn(5,70,dtype=torch.float64)
        after=before.clone();after[:,10:16]+=.1
        actual=integrated_change(model,before,after,32)
        delta=(.25*(model(after)[:,OUT]-model(before)[:,OUT])).detach().numpy()
        np.testing.assert_allclose(actual.sum(2),delta,atol=1e-7)
        np.testing.assert_array_equal(actual[:,:,:10],0)
        np.testing.assert_array_equal(actual[:,:,16:],0)

    def test_group_partition_and_signed_cancellation(self):
        self.assertEqual(sorted(i for ids in GROUPS.values() for i in ids),list(range(70)))
        c=np.array([[[2.,-1.]], [[-2.,1.]]])
        result=summarize_contributions(c,c.sum(2),['amplifying','opposing'])
        self.assertEqual(result['amplifying']['signed_covariance_share'],[2.])
        self.assertEqual(result['opposing']['signed_covariance_share'],[-1.])


if __name__=='__main__':
    unittest.main()
