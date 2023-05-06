import { useEffect, useState } from 'react';
import './App.css';
import AVAILABLE_LOG_TYPES from "./LogTypes.js";

import Tabs from './Tabs';
import Backend from './Backend';


function App() {

  const [connected, setConnected] = useState(false);
  const [lastLogBlockTimeStamp, setLastLogBlockTimeStamp] = useState('');
  const [params, setParams] = useState({}); // Name: {param}
  const [displayParams, setDisplayParams] = useState({}); // Name: {param}
  const [plotParams, setPlotParams] = useState({}); // Name: {param}

  const connect = () => {
    setConnected(true);
    fetchLogBlocks();
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
    console.log(displayParams);
  }, [displayParams]);

  useEffect(() => {
    fetchLogBlocks();
  }, [connected]);


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

  const fetchLogBlocks = async () => {
    if (connected) {
      let log_blocks = await Backend.fetchLogBlocks();

      if (log_blocks.length > 0) {
        let last_block = log_blocks[log_blocks.length-1];
        setLastLogBlockTimeStamp(last_block.header.timestamp);

        for (const [param_name, param_val] of Object.entries(last_block.data)) {
          params[param_name].value = param_val;
        }
      }

      //for (let block of log_blocks) {
      //  console.log(block);
      //}

      if (connected) {
        setTimeout(() => fetchLogBlocks(), 1000);
      }
    }
  };


  const updateParamValues = log_blocks => {
    for (let block of log_blocks) {
    }
  }

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
          />
        </div>
      </div>

    </div>
  );
}

export default App;
