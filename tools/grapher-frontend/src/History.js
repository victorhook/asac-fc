function HistoryBlock({ block }) {
    return (
        <div className="row">
            { block }
        </div>
    )
}

function History({ params, displayHistory, plotHistory }) {
    return (
        <div>
            <h5>History</h5>
            <hr />
            <table>
                <thead>
                    {
                        Object.values(params).filter(p => p.displayChecked)
                        .map(p => 
                            <th>{ p.name }</th>
                            )
                    }
                </thead>
            </table>
            <ul>
                {displayHistory.map(block => 
                    <HistoryBlock block={block} />
                )}
            </ul>
            <h3>{displayHistory.length}</h3>
        </div>
    )
}


export default History;
