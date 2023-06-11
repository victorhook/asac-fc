import './ControlHeader.css';
import Backend from '../Backend';

function ControlHeader() {

    return <div className='control-header'>
        <input className='button' type="file" onChange={Backend.readFile}/>
        <button className='button' onClick={Backend.saveSettings}>Save Settings</button>
    </div>
}

export default ControlHeader;