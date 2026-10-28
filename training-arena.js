/* Static asset adapter for the existing Spell Blade WebGL2 engine. Metres, Y up.
   Generated geometry includes glTF node transforms; collision-only meshes never draw. */
window.TrainingArena = (() => {
  'use strict';
  const base = 'assets/training/';
  let promise, data, gpu, engine;
  const cells = new Map(), floors = [], walls = [];
  const key = (x,z) => `${Math.floor(x)},${Math.floor(z)}`;
  const vec = (a,b) => a.map((v,i)=>v-b[i]);
  const cross = (a,b) => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  function addTriangle(a,b,c) {
    const n=cross(vec(b,a),vec(c,a)), len=Math.hypot(...n);
    if (len<1e-8) return;
    const t={a,b,c,n:n.map(x=>x/len),minY:Math.min(a[1],b[1],c[1]),maxY:Math.max(a[1],b[1],c[1])};
    const floor=t.n[1]>.55; (floor?floors:walls).push(t);
    for(let x=Math.floor(Math.min(a[0],b[0],c[0])-.32);x<=Math.floor(Math.max(a[0],b[0],c[0])+.32);x++)
      for(let z=Math.floor(Math.min(a[2],b[2],c[2])-.32);z<=Math.floor(Math.max(a[2],b[2],c[2])+.32);z++) {
        const k=key(x,z); if(!cells.has(k)) cells.set(k,{floor:[],wall:[]}); cells.get(k)[floor?'floor':'wall'].push(t);
      }
  }
  function prepareCollision(a) {
    cells.clear(); floors.length=walls.length=0;
    for(let i=0;i<a.length;i+=9) addTriangle(Array.from(a.slice(i,i+3)),Array.from(a.slice(i+3,i+6)),Array.from(a.slice(i+6,i+9)));
    // Same explicit landing end/side bounds as the approved Godot wrapper.
    function box(x,z,sx,sz){const y0=-.1,y1=3; const v=[[x-sx/2,y0,z-sz/2],[x+sx/2,y0,z-sz/2],[x+sx/2,y0,z+sz/2],[x-sx/2,y0,z+sz/2]];
      for(let i=0;i<4;i++){let a=v[i],b=v[(i+1)%4],c=[b[0],y1,b[2]],d=[a[0],y1,a[2]];addTriangle(a,b,c);addTriangle(a,c,d);}}
    for(const s of [-1,1]){box(s*22.45,0,.2,4.4);box(s*21.1,-2.1,2.9,.2);box(s*21.1,2.1,2.9,.2);}
  }
  function atTriangle(t,x,z) {
    const {a,b,c}=t, den=(b[2]-c[2])*(a[0]-c[0])+(c[0]-b[0])*(a[2]-c[2]);
    if(Math.abs(den)<1e-9)return null;
    const u=((b[2]-c[2])*(x-c[0])+(c[0]-b[0])*(z-c[2]))/den;
    const v=((c[2]-a[2])*(x-c[0])+(a[0]-c[0])*(z-c[2]))/den;
    return u>=-1e-5&&v>=-1e-5&&u+v<=1.00001 ? u*a[1]+v*b[1]+(1-u-v)*c[1]:null;
  }
  function height(x,z,reference=0) {
    let best=null;
    for(const t of cells.get(key(x,z))?.floor||[]) {
      if(t.minY>reference+.32||t.maxY<reference-.7)continue;
      const h=atTriangle(t,x,z);
      if(h!==null&&h<=reference+.32&&h>=reference-.7&&(best===null||h>best))best=h;
    }
    return best;
  }
  function distanceSegment(x,z,a,b) {const dx=b[0]-a[0],dz=b[2]-a[2],d=dx*dx+dz*dz;
    const q=d?Math.max(0,Math.min(1,((x-a[0])*dx+(z-a[2])*dz)/d)):0;
    return Math.hypot(x-a[0]-q*dx,z-a[2]-q*dz);}
  function blocked(x,z,h) {
    for(const t of cells.get(key(x,z))?.wall||[]) {
      if(t.maxY<=h+.12||t.minY>=h+1.65)continue;
      // Clip triangle to capsule's vertical span before testing its XZ projection.
      let poly=[t.a,t.b,t.c];
      for(const [level,sign] of [[h+.12,1],[h+1.65,-1]]) {
        const out=[];
        for(let i=0;i<poly.length;i++){const a=poly[i],b=poly[(i+1)%poly.length],ia=(a[1]-level)*sign>=0,ib=(b[1]-level)*sign>=0;
          if(ia)out.push(a); if(ia!==ib){const k=(level-a[1])/(b[1]-a[1]);out.push(a.map((v,j)=>v+(b[j]-v)*k));}}
        poly=out;
      }
      if(poly.some((a,i)=>distanceSegment(x,z,a,poly[(i+1)%poly.length])<.29))return true;
    }
    // Physical footprint needs ground support, not just its centre.
    for(const [dx,dz] of [[.24,0],[-.24,0],[0,.24],[0,-.24]]) if(height(x+dx,z+dz,h)===null)return true;
    return false;
  }
  function move(fromX,fromZ,toX,toZ,h=0) {
    let x=fromX,z=fromZ,y=height(x,z,h)??h;
    const steps=Math.max(1,Math.ceil(Math.hypot(toX-x,toZ-z)/.10)),dx=(toX-x)/steps,dz=(toZ-z)/steps;
    function attempt(nx,nz){const ny=height(nx,nz,y);if(ny===null||blocked(nx,nz,ny))return false;x=nx;z=nz;y=ny;return true;}
    for(let i=0;i<steps;i++)if(!attempt(x+dx,z+dz)){attempt(x+dx,z);attempt(x,z+dz);}
    return {x,z,h:y};
  }
  function pick(E,nx,ny) {
    const b=E.basis,d=b.f.map((v,i)=>v+b.r[i]*nx*b.tanY*b.aspect+b.u[i]*ny*b.tanY);
    let nearest=Infinity,hit=null;
    for(const t of floors.concat(walls)) {
      const den=t.n.reduce((s,v,i)=>s+v*d[i],0); if(Math.abs(den)<1e-8)continue;
      const time=t.n.reduce((s,v,i)=>s+v*(t.a[i]-E.cam[i]),0)/den;
      if(time<=0||time>=nearest)continue;
      const p=E.cam.map((v,i)=>v+d[i]*time),v0=vec(t.b,t.a),v1=vec(t.c,t.a),v2=vec(p,t.a);
      const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),d00=dot(v0,v0),d01=dot(v0,v1),d11=dot(v1,v1),d20=dot(v2,v0),d21=dot(v2,v1),q=d00*d11-d01*d01;
      const v=(d11*d20-d01*d21)/q,w=(d00*d21-d01*d20)/q;
      if(v>=0&&w>=0&&v+w<=1){nearest=time;hit=t.n[1]>.55?{x:p[0],z:p[2],h:p[1]}:null;}
    }
    return hit;
  }
  async function fetchOK(url,type){const r=await fetch(base+url,{signal:AbortSignal.timeout(20000)});if(!r.ok)throw Error(`${url}: HTTP ${r.status}`);return r[type]();}
  async function pack(name){const [meta,bin]=await Promise.all([fetchOK(name+'.json','json'),fetchOK(name+'.bin','arrayBuffer')]);
    const read=(v,T)=>new T(bin,v[0],v[1]);
    return {...meta,records:meta.records.map(r=>({...r,p:read(r.p,Float32Array),n:read(r.n,Float32Array),u:read(r.u,Float32Array),ix:read(r.ix,Uint16Array)})),collision:read(meta.collision,Float32Array)};}
  const tintRules=[['sandstone 0',[.98,.84,.71]],['sandstone 1',[1,.94,.86]],['sandstone 2',[1,.99,.96]],['sandstone 3',[.92,.79,.67]],['sandstone 4',[1,.97,.89]],['foliage 0',[.31,.40,.18]],['foliage 1',[.40,.49,.23]],['foliage 2',[.52,.58,.30]],['vermilion cloth',[.64,.28,.22]],['teal cloth',[.24,.48,.49]],['ochre embroidery',[.77,.55,.30]],['flax rope',[.59,.46,.30]],['weathered iron',[.26,.25,.23]],['timber ends',[.46,.31,.20]]];
  function material(m,override){
    const p=m.pbrMetallicRoughness||{},label=(m.name||'').toLowerCase();
    const a={color:p.baseColorFactor||[1,1,1,1],rough:p.roughnessFactor??1,metal:p.metallicFactor??0,double:m.doubleSided||label.includes('foliage'),cut:m.alphaMode==='MASK'?(m.alphaCutoff??.5):0,blend:m.alphaMode==='BLEND',strength:1};
    if(!override)return a;
    for(const [s,c] of tintRules)if(label.includes(s))a.color=[...c,1];
    if(label.includes('golden sand')){a.color=[1,1,1,1];a.maps=['Sand_Peach_Albedo_2K','Sand_Raked_Footprints_Normal_2K','Sand_Matte_Roughness_2K'];a.strength=.75;}
    if(label.includes('sandstone')){a.maps=['Stone_WarmMineral_Albedo_512','Stone_SelectivePits_Normal_512','Stone_Matte_Roughness_512'];a.strength=.5;}
    if(label.includes('warm timber')){a.color=[1,1,1,1];a.maps=['Wood_Worn_Albedo_512','Wood_Grain_Normal_512','Wood_Dry_Roughness_512'];a.strength=.35;}
    if(label.includes('cloth')||label.includes('embroidery')){a.normal='Cloth_Weave_Normal_256.png';a.strength=.28;}
    a.rough=label.includes('weathered iron')?.8:.96;a.metal=label.includes('weathered iron')?.35:0;
    return a;
  }
  const VS=`#version 300 es
  layout(location=0) in vec3 aPos; layout(location=1) in vec3 aNor; layout(location=2) in vec2 aUv;
  uniform mat4 uVP,uShadowVP; uniform vec3 uOffset; out vec3 vN;out vec3 vW;out vec2 vUv;out vec4 vShadow;
  void main(){vW=aPos+uOffset;vN=aNor;vUv=aUv;vShadow=uShadowVP*vec4(vW,1.);gl_Position=uVP*vec4(vW,1.);}`;
  const FS=`#version 300 es
  precision highp float;in vec3 vN;in vec3 vW;in vec2 vUv;in vec4 vShadow;out vec4 o;
  uniform vec4 uColor;uniform vec3 uCam;uniform sampler2D uBase;uniform sampler2D uNormal;uniform sampler2D uRough;
  uniform float uHasBase,uHasNormal,uHasRough,uStrength,uRoughness,uMetal,uCut,uFlash;
  uniform sampler2D uShadow;
  uniform vec3 uTorches[3];
  vec3 linearColor(vec3 c){return mix(c/12.92,pow((c+.055)/1.055,vec3(2.4)),step(vec3(.04045),c));}
  vec3 srgb(vec3 c){return mix(c*12.92,1.055*pow(max(c,vec3(0.)),vec3(1./2.4))-.055,step(vec3(.0031308),c));}
  void main(){vec4 tex=uHasBase>.5?texture(uBase,vUv):vec4(1.);float alpha=tex.a*uColor.a;if(alpha<uCut)discard;
    vec3 base=linearColor(tex.rgb)*uColor.rgb,n=normalize(vN);if(!gl_FrontFacing)n=-n;
    if(uHasNormal>.5){vec3 q1=dFdx(vW),q2=dFdy(vW);vec2 s1=dFdx(vUv),s2=dFdy(vUv);
      vec3 T=q1*s2.y-q2*s1.y,B=-q1*s2.x+q2*s1.x;float inv=inversesqrt(max(dot(T,T),dot(B,B))+1e-10)*sign(s1.x*s2.y-s1.y*s2.x);
      vec3 nm=texture(uNormal,vUv).xyz*2.-1.;nm.xy*=uStrength;n=normalize(mat3(T*inv,B*inv,n)*nm);}
    float rough=clamp(uRoughness*(uHasRough>.5?texture(uRough,vUv).r:1.),.12,1.);
    vec3 L=normalize(vec3(-.608,.669,.426)),V=normalize(uCam-vW),H=normalize(L+V);
    float sun=max(dot(n,L),0.),sky=n.y*.5+.5;
    vec3 fill=mix(vec3(.35,.30,.25),vec3(.58,.56,.53),sky)*.85;
    vec3 shadowPoint=vShadow.xyz/vShadow.w*.5+.5;float shade=0.;
    for(int x=-1;x<=1;x++)for(int y=-1;y<=1;y++){
      float depth=texture(uShadow,shadowPoint.xy+vec2(float(x),float(y))/2048.).r;
      shade+=shadowPoint.z-.0012<=depth?1.:0.;
    }
    shade/=9.;if(any(lessThan(shadowPoint,vec3(0.)))||any(greaterThan(shadowPoint,vec3(1.))))shade=1.;
    vec3 F=mix(vec3(.04),base,uMetal);float sp=pow(max(dot(n,H),0.),mix(120.,3.,rough))*mix(.5,.035,rough);
    vec3 col=base*(fill+vec3(1.,.94,.86)*sun*.95*shade)+F*sp;
    for(int i=0;i<3;i++){vec3 delta=uTorches[i]-vW;float len=length(delta);float fall=pow(max(0.,1.-len/3.8),2.);col+=base*vec3(1.,.48,.14)*.35*fall*max(dot(n,delta/max(len,.01)),0.);}
    col=mix(srgb(col*1.4),vec3(1.),uFlash);
    float fog=pow(clamp((length(uCam-vW)-35.)/115.,0.,1.),1.4);o=vec4(mix(col,vec3(.83,.75,.7),fog),alpha);
  }`;
  function shader(gl,type,source){const s=gl.createShader(type);gl.shaderSource(s,source);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(s));return s;}
  async function upload(E) {
    const gl=E.gl, resources={buffers:[],vaos:[],textures:[],shaders:[],programs:[],fbos:[]};
    const destroy=()=>{resources.buffers.forEach(x=>gl.deleteBuffer(x));resources.vaos.forEach(x=>gl.deleteVertexArray(x));resources.textures.forEach(x=>gl.deleteTexture(x));resources.shaders.forEach(x=>gl.deleteShader(x));resources.programs.forEach(x=>gl.deleteProgram(x));resources.fbos.forEach(x=>gl.deleteFramebuffer(x));};
    try{
      const pr=gl.createProgram();resources.programs.push(pr);
      for(const [type,src] of [[gl.VERTEX_SHADER,VS],[gl.FRAGMENT_SHADER,FS]]){const s=shader(gl,type,src);resources.shaders.push(s);gl.attachShader(pr,s);}gl.linkProgram(pr);if(!gl.getProgramParameter(pr,gl.LINK_STATUS))throw Error(gl.getProgramInfoLog(pr));
      const u={};for(const s of ['VP','ShadowVP','Shadow','Offset','Color','Cam','Base','Normal','Rough','HasBase','HasNormal','HasRough','Strength','Roughness','Metal','Cut','Flash'])u[s]=gl.getUniformLocation(pr,'u'+s);
      u.Torches=gl.getUniformLocation(pr,'uTorches[0]');
      const cache=new Map();
      async function texture(name){if(!name)return null;if(cache.has(name))return cache.get(name);
        const pending=(async()=>{const r=await fetchOK(name,'blob'),img=await createImageBitmap(r,{premultiplyAlpha:'none',colorSpaceConversion:'none'});
          const t=gl.createTexture();resources.textures.push(t);gl.bindTexture(gl.TEXTURE_2D,t);gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL,false);gl.texImage2D(gl.TEXTURE_2D,0,gl.RGBA,gl.RGBA,gl.UNSIGNED_BYTE,img);img.close();
          gl.generateMipmap(gl.TEXTURE_2D);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.LINEAR_MIPMAP_LINEAR);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.LINEAR);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_S,gl.REPEAT);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_T,gl.REPEAT);
          const ext=gl.getExtension('EXT_texture_filter_anisotropic');if(ext)gl.texParameterf(gl.TEXTURE_2D,ext.TEXTURE_MAX_ANISOTROPY_EXT,Math.min(8,gl.getParameter(ext.MAX_TEXTURE_MAX_ANISOTROPY_EXT)));return t;})();cache.set(name,pending);return pending;}
      async function model(pack,override){const result=[];for(const r of pack.records){const a=material(r.material,override),p=r.material.pbrMetallicRoughness||{},image=i=>i===undefined?null:pack.images[pack.textures[i].source];
          const names=a.maps?a.maps.map(n=>n+'.png'):[image(p.baseColorTexture?.index),a.normal||image(r.material.normalTexture?.index),null];
          const pending=await Promise.allSettled(names.map(texture));
          const failed=pending.find(x=>x.status==='rejected');if(failed)throw failed.reason;
          const textures=pending.map(x=>x.value),vao=gl.createVertexArray();resources.vaos.push(vao);gl.bindVertexArray(vao);
          for(const [loc,arr,size] of [[0,r.p,3],[1,r.n,3],[2,r.u,2]]){const b=gl.createBuffer();resources.buffers.push(b);gl.bindBuffer(gl.ARRAY_BUFFER,b);gl.bufferData(gl.ARRAY_BUFFER,arr,gl.STATIC_DRAW);gl.enableVertexAttribArray(loc);gl.vertexAttribPointer(loc,size,gl.FLOAT,false,0,0);}
          const ib=gl.createBuffer();resources.buffers.push(ib);gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER,ib);gl.bufferData(gl.ELEMENT_ARRAY_BUFFER,r.ix,gl.STATIC_DRAW);result.push({vao,count:r.ix.length,a,textures});}
        gl.bindVertexArray(null);return result;}
      // Serial GPU mutations; texture fetching is cached across both models.
      const arena=await model(data.arena,true),dummy=await model(data.dummy,false);
      // Static warm-sun depth pass. GLB lights are deliberately not assumed.
      const norm=a=>{let n=Math.hypot(...a);return a.map(x=>x/n);},z=norm([-.608,.669,.426]),x=norm(cross([0,1,0],z)),y=cross(z,x);
      const shadowVP=new Float32Array([x[0]/32,y[0]/32,-z[0]/70,0,x[1]/32,y[1]/32,-z[1]/70,0,x[2]/32,y[2]/32,-z[2]/70,0,0,0,0,1]);
      const shadow=gl.createTexture();resources.textures.push(shadow);gl.bindTexture(gl.TEXTURE_2D,shadow);
      gl.texImage2D(gl.TEXTURE_2D,0,gl.DEPTH_COMPONENT24,2048,2048,0,gl.DEPTH_COMPONENT,gl.UNSIGNED_INT,null);
      for(const [p,v] of [[gl.TEXTURE_MIN_FILTER,gl.NEAREST],[gl.TEXTURE_MAG_FILTER,gl.NEAREST],[gl.TEXTURE_WRAP_S,gl.CLAMP_TO_EDGE],[gl.TEXTURE_WRAP_T,gl.CLAMP_TO_EDGE]])gl.texParameteri(gl.TEXTURE_2D,p,v);
      const fb=gl.createFramebuffer();resources.fbos.push(fb);gl.bindFramebuffer(gl.FRAMEBUFFER,fb);gl.framebufferTexture2D(gl.FRAMEBUFFER,gl.DEPTH_ATTACHMENT,gl.TEXTURE_2D,shadow,0);gl.drawBuffers([gl.NONE]);gl.readBuffer(gl.NONE);
      if(gl.checkFramebufferStatus(gl.FRAMEBUFFER)!==gl.FRAMEBUFFER_COMPLETE)throw Error('Training shadow framebuffer unavailable');
      const depth=gl.createProgram();resources.programs.push(depth);
      for(const [type,src] of [[gl.VERTEX_SHADER,VS],[gl.FRAGMENT_SHADER,'#version 300 es\nprecision mediump float;void main(){}']]){const sh=shader(gl,type,src);resources.shaders.push(sh);gl.attachShader(depth,sh);}gl.linkProgram(depth);if(!gl.getProgramParameter(depth,gl.LINK_STATUS))throw Error(gl.getProgramInfoLog(depth));
      gl.useProgram(depth);gl.uniformMatrix4fv(gl.getUniformLocation(depth,'uVP'),false,shadowVP);gl.uniform3f(gl.getUniformLocation(depth,'uOffset'),0,0,0);
      gl.viewport(0,0,2048,2048);gl.enable(gl.DEPTH_TEST);gl.depthMask(true);gl.clearDepth(1);gl.clear(gl.DEPTH_BUFFER_BIT);gl.disable(gl.CULL_FACE);
      for(const r of arena.concat(dummy)){gl.bindVertexArray(r.vao);gl.drawElements(gl.TRIANGLES,r.count,gl.UNSIGNED_SHORT,0);}
      gl.bindFramebuffer(gl.FRAMEBUFFER,null);gl.bindVertexArray(null);gl.enable(gl.CULL_FACE);E.cur=null;
      return {pr,u,arena,dummy,destroy,shadow,shadowVP};
    }catch(e){gl.bindFramebuffer(gl.FRAMEBUFFER,null);gl.bindVertexArray(null);E.cur=null;destroy();throw e;}
  }
  async function load(E) {
    if(gpu&&engine===E)return;
    if(promise)return promise;
    promise=(async()=>{if(!data){const [arena,dummy]=await Promise.all([pack('spellblade_arena'),pack('training_dummy')]);data={arena,dummy};prepareCollision(arena.collision);}gpu=await upload(E);engine=E;})().catch(e=>{promise=null;throw e;});
    return promise;
  }
  function draw(E,which='arena',offset=[0,0,0],flash=0) {
    if(!gpu||engine!==E)return false;
    const gl=E.gl,{pr,u}=gpu;gl.useProgram(pr);E.cur=null;gl.uniformMatrix4fv(u.VP,false,E.VP);gl.uniform3fv(u.Offset,offset);gl.uniform3fv(u.Cam,E.cam);gl.uniform1f(u.Flash,flash);
    gl.uniformMatrix4fv(u.ShadowVP,false,gpu.shadowVP);gl.activeTexture(gl.TEXTURE3);gl.bindTexture(gl.TEXTURE_2D,gpu.shadow);gl.uniform1i(u.Shadow,3);
    const near=torches.map(p=>({p,d:Math.hypot(p[0]-(E.trainingPlayer?.[0]||0),p[2]-(E.trainingPlayer?.[2]||0))})).filter(x=>x.d<10).sort((a,b)=>a.d-b.d).slice(0,3).map(x=>x.p);
    while(near.length<3)near.push([0,-10000,0]);gl.uniform3fv(u.Torches,near.flat());
    for(const r of gpu[which]){const a=r.a;a.double?gl.disable(gl.CULL_FACE):gl.enable(gl.CULL_FACE);if(a.blend){gl.enable(gl.BLEND);gl.blendFunc(gl.SRC_ALPHA,gl.ONE_MINUS_SRC_ALPHA);gl.depthMask(false);}
      gl.uniform4fv(u.Color,a.color);gl.uniform1f(u.Roughness,a.rough);gl.uniform1f(u.Metal,a.metal);gl.uniform1f(u.Strength,a.strength);gl.uniform1f(u.Cut,a.cut);
      for(let i=0;i<3;i++){gl.activeTexture(gl.TEXTURE0+i);gl.bindTexture(gl.TEXTURE_2D,r.textures[i]);gl.uniform1i([u.Base,u.Normal,u.Rough][i],i);gl.uniform1f([u.HasBase,u.HasNormal,u.HasRough][i],r.textures[i]?1:0);}
      gl.bindVertexArray(r.vao);gl.drawElements(gl.TRIANGLES,r.count,gl.UNSIGNED_SHORT,0);if(a.blend){gl.disable(gl.BLEND);gl.depthMask(true);}}
    gl.bindVertexArray(null);gl.activeTexture(gl.TEXTURE0);gl.enable(gl.CULL_FACE);return true;
  }
  const torches=[];
  for(let i=0;i<12;i++){if(i%3!==1){const a=Math.PI*2*(i+.5)/12;torches.push([Math.cos(a)*14.55,2.91,-Math.sin(a)*14.55]);}}
  function drawTorches(E,t) {
    for(let i=0;i<12;i++){
      if(i%3===1)continue;
      const a=Math.PI*2*(i+.5)/12,x=Math.cos(a)*14.55,z=-Math.sin(a)*14.55;
      const flicker=.95+.06*Math.sin(t*2.1+i)+.04*Math.sin(t*3.8+i*2);
      E.push();E.T(x,2.56,z);E.Sc(.18*flicker,.70*flicker,.18*flicker);E.flat('cone',[1,.36,.055,.85],{});E.pop();
      E.push();E.T(x,2.56,z);E.Sc(.09*flicker,.46*flicker,.09*flicker);E.flat('cone',[1,.85,.28,.95],{});E.pop();
    }
  }
  return {load,draw,drawTorches,height,move,pick,get ready(){return !!gpu;},get diagnostics(){return {floors:floors.length,walls:walls.length,arena:gpu?.arena.length,dummy:gpu?.dummy.length,textures:gpu?.arena.filter(x=>x.textures[1]).length};}};
})();
