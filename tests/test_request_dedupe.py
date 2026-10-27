"""Idempotent ROS2 request cache (pure Python) prevents duplicate Nav2 goals."""
import unittest
from oomwoo_cleaning_jobs.request_dedupe import RequestDedupe,CommandReplayError


class RequestIdempotenceTests(unittest.TestCase):
    def setUp(self):
        self.cache=RequestDedupe(8)
        self.sends=[]

    def send(self,request):
        self.sends.append(request)
        return {"ok":True,"goal_token":"NAV-1","op":request["op"]}

    def test_duplicate_exact_request_invokes_handler_once(self):
        req={"op":"start","job_id":"test"}
        a=self.cache.handle("id-1",req,self.send)
        b=self.cache.handle("id-1",dict(req),self.send)
        self.assertEqual(a,b)
        self.assertEqual(len(self.sends),1)

    def test_reused_id_with_other_semantics_refused(self):
        self.cache.handle("id-1",{"op":"pause"},self.send)
        with self.assertRaises(CommandReplayError):
            self.cache.handle("id-1",{"op":"resume"},self.send)
        self.assertEqual(len(self.sends),1)

    def test_matching_json_key_order_is_same_command(self):
        self.cache.handle("id-1",{"a":1,"op":"start"},self.send)
        self.cache.handle("id-1",{"op":"start","a":1},self.send)
        self.assertEqual(len(self.sends),1)

    def test_bounded_cache_does_not_grow_unbounded(self):
        for i in range(50):
            self.cache.handle("r-"+str(i),{"op":"status","n":i},self.send)
        self.assertEqual(self.cache.size(),8)

    def test_invalid_request_ids_rejected(self):
        for key in ("","spaces bad","/unsafe","u"*101,None):
            with self.subTest(key=key),self.assertRaises(CommandReplayError):
                self.cache.handle(key,{"op":"pause"},self.send)
        self.assertEqual(len(self.sends),0)

    def test_nonfinite_command_refused(self):
        with self.assertRaises(CommandReplayError):
            self.cache.handle("id-1",{"op":"start","x":float("nan")},self.send)

    def test_duplicate_rejected_result_stays_rejected(self):
        calls=[]
        def reject(msg):
            calls.append(msg)
            return {"ok":False,"error":"map missing"}
        r={"op":"start"}
        self.assertFalse(self.cache.handle("start1",r,reject)["ok"])
        self.assertFalse(self.cache.handle("start1",r,reject)["ok"])
        self.assertEqual(len(calls),1)

    def test_bad_handler_result_is_not_cached(self):
        with self.assertRaises(CommandReplayError):
            self.cache.handle("x",{"op":"none"},lambda _: {"not_ok":True})
        self.assertEqual(self.cache.size(),0)

    def test_caller_cannot_mutate_cached_outcome(self):
        a=self.cache.handle("test",{"op":"start"},self.send)
        a["goal_token"]="mutated"
        b=self.cache.handle("test",{"op":"start"},self.send)
        self.assertEqual(b["goal_token"],"NAV-1")

if __name__=="__main__":
    unittest.main()
