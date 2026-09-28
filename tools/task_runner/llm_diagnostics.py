"""Bounded metadata only: never persist request/response bodies or trace info."""
import copy
import json
import re
import time

PHASES = {'preparing':'准备请求','connecting':'连接服务','tls':'建立加密连接',
          'uploading':'上传请求','waiting_headers':'等待响应头','reading_body':'读取响应正文',
          'parsing':'校验模型响应','completed':'已完成'}


class RequestDiagnostics:
    def __init__(self, payload, encoded, timeout, proxy, callback=None):
        self.started=time.monotonic();self.callback=callback;self.proxy=bool(proxy);self.tls_ready=False
        self.data={'version':1,'model':payload['model'],'reasoning_effort':payload['reasoning']['effort'],
            'timeout_s':timeout,'stream':payload['stream'],'proxy_enabled':self.proxy,
            'request_bytes':len(encoded),'input_items':len(payload['input']),'tool_count':len(payload['tools']),
            'input_json_bytes':len(self.encode(payload['input'])),
            'tools_json_bytes':len(self.encode(payload['tools'])),
            'image_count':sum(c.get('type')=='input_image' for item in payload['input']
                              for c in (item.get('content',[]) if isinstance(item.get('content'),list) else []) if isinstance(c,dict)),
            'response_bytes':0,'phase':'preparing','transport_events':[]}
        self.publish()

    @staticmethod
    def encode(value):return json.dumps(value,ensure_ascii=False,separators=(',',':'),allow_nan=False).encode()

    def elapsed(self):return round(time.monotonic()-self.started,3)

    def publish(self):
        if self.callback:self.callback(copy.deepcopy(self.data))

    def mark(self,phase,**values):
        self.data.update(phase=phase,elapsed_s=self.elapsed(),**values);self.publish()

    async def trace(self,name,info):
        # info may contain credentials, URLs, TLS objects and exceptions. Never inspect/store it.
        protocol,_,event=name.partition('.')
        allowed={'connect_tcp','start_tls','send_request_headers','send_request_body','receive_response_headers','receive_response_body','response_closed'}
        stage,_,outcome=event.rpartition('.')
        if protocol not in ('connection','proxy','http11','http2') or stage not in allowed or outcome not in ('started','complete','failed'):return
        rows=self.data['transport_events']
        if len(rows)<48:rows.append({'event':name,'seconds':self.elapsed()})
        else:self.data['transport_events_dropped']=self.data.get('transport_events_dropped',0)+1
        if stage=='connect_tcp':
            self.data['phase']='connecting'
            if outcome=='complete':self.data['connect_complete_s']=self.elapsed()
        elif stage=='start_tls':
            self.data['phase']='tls'
            if outcome=='complete':self.tls_ready=True;self.data['tls_complete_s']=self.elapsed()
        elif not self.proxy or self.tls_ready:
            if stage in ('send_request_headers','send_request_body'):
                self.data['phase']='uploading'
                if stage=='send_request_body' and outcome=='complete':
                    self.data.update(request_sent_s=self.elapsed(),phase='waiting_headers')
            elif stage=='receive_response_headers':self.data['phase']='waiting_headers'
            elif stage=='receive_response_body':self.data['phase']='reading_body'
        self.data['elapsed_s']=self.elapsed();self.publish()

    def headers(self,response):
        values={'http_status':response.status_code,'response_headers_s':self.elapsed()}
        for key in ('x-request-id','request-id'):
            value=response.headers.get(key,'')
            if re.fullmatch(r'[A-Za-z0-9_.:-]{1,160}',value):values['provider_request_id']=value;break
        self.mark('reading_body',**values)

    def chunk(self,part):
        self.data['response_bytes']+=len(part)
        if part and 'first_byte_s' not in self.data:self.mark('reading_body',first_byte_s=self.elapsed())

    def response(self,value):
        if not isinstance(value,dict):return
        if value.get('status') in ('completed','incomplete','failed','cancelled','queued','in_progress'):
            self.data['response_status']=value['status']
        usage=value.get('usage') or {};counts={}
        if isinstance(usage,dict):
            for key in ('input_tokens','output_tokens','total_tokens'):
                if type(usage.get(key)) is int and usage[key]>=0:counts[key]=usage[key]
            for group,key in [('input_tokens_details','cached_tokens'),('output_tokens_details','reasoning_tokens')]:
                details=usage.get(group) or {}
                if isinstance(details,dict) and type(details.get(key)) is int and details[key]>=0:counts[key]=details[key]
        if counts:self.data['usage']=counts

    def finish(self,outcome,error=None):
        values={'outcome':outcome,'total_s':self.elapsed()}
        if error is not None:values['exception_type']=type(error).__name__
        self.data.update(values);self.mark('completed' if outcome=='completed' else self.data['phase'])

    def failure_message(self,timeout):
        what=f'模型请求超过{self.data["timeout_s"]:g}秒时限' if timeout else '模型网络请求失败'
        return f'{what}（阶段：{PHASES[self.data["phase"]]}）；未自动重试'
