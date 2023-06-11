import { useEffect, useState } from 'react';
import './App.css';
import AVAILABLE_LOG_TYPES from "./LogTypes.js";

import Tabs from './Tabs';
import Backend from './Backend';


function App() {

  const [connected, setConnected] = useState(false);
  const [apiFetchInterval, setApiFetchInterval] = useState(null);
  const [lastLogBlockTimeStamp, setLastLogBlockTimeStamp] = useState('');
  const [params, setParams] = useState({}); // Name: {param}
  const [lastLogBlock, setLastLogBlock] = useState({}); // Name: {param}
  const [displayHistory, setDisplayHistory] = useState([]);
  const [plotHistory, setPlotHistory] = useState([]);
  const [displayParams, setDisplayParams] = useState({}); // Name: {param}
  const [plotParams, setPlotParams] = useState({}); // Name: {param}

  const connect = () => {
    setConnected(true);
  };

  const disconnect = () => {
    setConnected(false);
  };

  const save = () => {

  };

  useEffect(() => {
    setDisplayParams(Object.fromEntries(
      Object.entries(params).filter(([param_name, param]) => param.displayChecked)
    ));
    setPlotParams(Object.fromEntries(
      Object.entries(params).filter(([param_name, param]) => param.plotChecked)
    ));
  }, [params]);

  useEffect(() => {
    //console.log(displayParams);
  }, [displayParams]);

  useEffect(() => {
    let new_params = {};

    for (let param of [...AVAILABLE_LOG_TYPES]) {
        param.displayChecked = false;
        param.plotChecked = false;
        param.value = 0;
        new_params[param.name] = param;
    }
    setParams(new_params);
  }, []);

  const fetchLogBlocks = () => {
    Backend.fetchLogBlocks()
    .then(res => {
      if (res.status == 'OK') {
        // We only add the data we're plotting/displaying
        console.log(res.data, displayParams);
        //console.log(res.data.filter(p => p.name in displayParams));
        setDisplayHistory(curr => curr.concat(res.data.filter(p => p.name in Object.keys(displayParams))));
      } else {
        setConnected(false);
      }
    })
  };

  useEffect(() => {
    //console.log('HISTORY: ', displayHistory);
  }, [displayHistory]);

  useEffect(() => {
    if (connected) {
      console.log('WE ARE CONNECTED!');
      //setInterval(fetchLogBlocks, 1000);
      fetchLogBlocks();
      if (apiFetchInterval == null) {
        setApiFetchInterval(setInterval(fetchLogBlocks, 1000));
      }
    } else {
      if (apiFetchInterval != null) {
        clearInterval(apiFetchInterval);
        setApiFetchInterval(null);
      }
      console.log('WE ARE DISCONNECTED!');
    }
  }, [connected]);



  return (
    <div className="container-fluid">
      <nav className="navbar navbar-expand-lg navbar-light bg-light">
        <a className="nav-brand nav-link fs-2 m-1 me-5" href="/">
          ASAC Plotter
        </a>

        <button className="btn m-1" disabled={connected} onClick={connect}>Connect</button>
        <button className="btn m-1" disabled={!connected} onClick={disconnect}>Disconnect</button>
        <button className="btn m-1" onClick={save}>Save</button>
        <span className="ms-5" style={{color: connected ? 'green' : 'red'}}>
          {connected ? 'Connected' : 'Not connected'}
        </span>
        <div className="ms-5 d-flex align-center">
          <span className="fs-5">Last log block:</span>
          <span className="ms-2 fs-5">{ lastLogBlockTimeStamp }</span>
        </div>
      </nav>

      <div className="row">
        <div className="col-12">
          <Tabs params={params}
                setParams={setParams}
                displayParams={displayParams}
                plotParams={plotParams}
                displayHistory={displayHistory}
                plotHistory={plotHistory}
          />
        </div>
      </div>

    </div>
  );
}

export default App;
