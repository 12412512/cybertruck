import unittest,httpx
from fastapi import HTTPException
from robot.web.workbench import DeviceClient
class DeviceHttpTests(unittest.TestCase):
 def test_success_and_error_never_replayed(self):
  calls=[]
  def handler(r):
   calls.append(r)
   if r.method=='POST':raise httpx.ReadTimeout('unknown outcome',request=r)
   return httpx.Response(200,json={'connected':True})
  c=DeviceClient();c.http.close();c.http=httpx.Client(transport=httpx.MockTransport(handler))
  self.assertTrue(c.request('/state')['connected'])
  with self.assertRaises(HTTPException):c.request('/command',{'op':'mechanism'})
  self.assertEqual([r.method for r in calls],['GET','POST']);c.http.close()
if __name__=='__main__':unittest.main()
